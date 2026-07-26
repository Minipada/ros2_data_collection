// Port of dc_bridge_core uploader_tests: exercises the Uploader against an in-memory
// ObjectStore fake with failure injection (mirroring the Rust `Instrumented` wrapper
// over object_store's InMemory) — no aws-sdk-cpp / cloud dependency.
#include "dc_bridge/uploader/uploader.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <vector>

using namespace dc_bridge;
using namespace dc_bridge::uploader;
using nlohmann::json;

namespace
{

class InMemoryStore : public ObjectStore
{
public:
  std::optional<std::uint64_t> head(const std::string& key) override
  {
    if (fail_next_heads.fetch_sub(1) > 0)
    {
      throw ObjectStoreError("injected head failure");
    }
    std::lock_guard<std::mutex> lock(mu_);
    auto it = objects_.find(key);
    if (it == objects_.end())
    {
      return std::nullopt;
    }
    return static_cast<std::uint64_t>(it->second.size());
  }

  void put(const std::string& key, const std::string& bytes) override
  {
    if (fail_put_keys.count(key))
    {
      throw ObjectStoreError("injected put failure");
    }
    puts.fetch_add(1);
    std::lock_guard<std::mutex> lock(mu_);
    objects_[key] = bytes;
  }

  std::string create_multipart(const std::string& key) override
  {
    multipart_creates.fetch_add(1);
    std::lock_guard<std::mutex> lock(mu_);
    std::string id = "upload-" + std::to_string(next_id_++);
    uploads_[id] = Upload{ key, {} };
    return id;
  }

  std::string put_part(const std::string& /*key*/, const std::string& upload_id, int part_number,
                       const std::string& bytes) override
  {
    if (allow_parts.fetch_sub(1) <= 0)
    {
      throw ObjectStoreError("injected put_part failure");
    }
    std::lock_guard<std::mutex> lock(mu_);
    uploads_[upload_id].parts[part_number] = bytes;
    part_puts.push_back(part_number - 1);  // record 0-based, matching the Rust test
    return "etag-" + std::to_string(part_number);
  }

  void complete_multipart(const std::string& key, const std::string& upload_id,
                          const std::vector<std::string>& /*etags*/) override
  {
    std::lock_guard<std::mutex> lock(mu_);
    std::string all;
    for (auto& [part_number, data] : uploads_[upload_id].parts)
    {  // std::map = sorted
      all += data;
    }
    objects_[key] = all;
    uploads_.erase(upload_id);
  }

  std::string object_bytes(const std::string& key)
  {
    std::lock_guard<std::mutex> lock(mu_);
    return objects_.at(key);
  }
  bool has_object(const std::string& key)
  {
    std::lock_guard<std::mutex> lock(mu_);
    return objects_.count(key) > 0;
  }

  // Instrumentation
  std::atomic<int> puts{ 0 };
  std::atomic<int> multipart_creates{ 0 };
  std::atomic<long> fail_next_heads{ 0 };
  std::atomic<long> allow_parts{ std::numeric_limits<long>::max() };
  std::set<std::string> fail_put_keys;
  std::vector<int> part_puts;

private:
  struct Upload
  {
    std::string key;
    std::map<int, std::string> parts;
  };
  std::mutex mu_;
  std::map<std::string, std::string> objects_;
  std::map<std::string, Upload> uploads_;
  int next_id_ = 0;
};

struct Fixture
{
  std::filesystem::path tmp;
  std::vector<std::shared_ptr<InMemoryStore>> stores;
  std::vector<Storage> storages;

  explicit Fixture(const std::vector<std::string>& names)
  {
    tmp = std::filesystem::temp_directory_path() /
          ("dc_uploader_test_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    std::filesystem::create_directories(tmp);
    for (const auto& name : names)
    {
      auto store = std::make_shared<InMemoryStore>();
      stores.push_back(store);
      storages.push_back(Storage{ name, "s3://" + name + "-bucket/", "", store });
    }
  }
  ~Fixture()
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp, ec);
  }

  UploaderConfig config(bool delete_when_sent)
  {
    UploaderConfig c((tmp / "state").string(), delete_when_sent);
    c.retry_backoff = std::chrono::milliseconds(0);
    return c;
  }
  Uploader uploader(bool delete_when_sent)
  {
    return Uploader(config(delete_when_sent), storages, [](const std::string&) { return std::nullopt; });
  }
  std::string write_file(const std::string& name, const std::string& content)
  {
    auto p = tmp / name;
    std::ofstream(p, std::ios::binary) << content;
    return p.string();
  }

  static std::atomic<int> counter_;
};
std::atomic<int> Fixture::counter_{ 0 };

const std::string JPEG_BYTES = std::string("\xff\xd8\xff\xe0", 4) + "fake-jpeg-body";

json camera_payload(const std::string& local_path, const std::vector<std::string>& storages)
{
  json remote = json::object();
  for (const auto& s : storages)
  {
    remote[s] = { { "raw", "cam/2026/img.jpg" } };
  }
  return json{ { "name", "camera" },
               { "robot_name", "robot1" },
               { "id", "r1" },
               { "local_paths", { { "raw", local_path } } },
               { "remote_paths", remote } };
}

// An EmitFn collecting rows into `rows`.
EmitFn collect_rows(std::shared_ptr<std::vector<json>> rows)
{
  return [rows](const json& row) { rows->push_back(row); };
}

std::vector<json> rows_of_kind(const std::vector<json>& rows, const std::string& kind)
{
  std::vector<json> out;
  for (const auto& r : rows)
  {
    if (r.value("kind", "") == kind)
    {
      out.push_back(r);
    }
  }
  return out;
}

}  // namespace

TEST(Uploader, HappyPathUploadsVerifiesEmitsStatusRows)
{
  Fixture fx({ "minio", "s3_archive" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  auto up = fx.uploader(false);
  auto rows = std::make_shared<std::vector<json>>();

  auto summary =
      up.process_record(camera_payload(local, { "minio", "s3_archive" }), "dc.measurement.camera", collect_rows(rows));

  EXPECT_EQ(summary.files, 1u);
  EXPECT_EQ(summary.verified, 1u);
  EXPECT_TRUE(summary.group_complete);
  for (auto& store : fx.stores)
  {
    EXPECT_EQ(store->object_bytes("cam/2026/img.jpg"), JPEG_BYTES);
  }
  EXPECT_EQ(rows_of_kind(*rows, "file_status").size(), 2u);
  EXPECT_EQ(rows_of_kind(*rows, "group_complete").size(), 1u);
  EXPECT_TRUE(std::filesystem::exists(local));
}

TEST(Uploader, MetadataRecordHasHumbleShape)
{
  Fixture fx({ "minio" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  auto up = fx.uploader(false);
  auto rows = std::make_shared<std::vector<json>>();
  up.process_record(camera_payload(local, { "minio" }), "dc.measurement.camera", collect_rows(rows));

  auto row = rows_of_kind(*rows, "file_status")[0];
  EXPECT_EQ(row["group_name"], "camera");
  EXPECT_EQ(row["robot_name"], "robot1");
  EXPECT_EQ(row["robot_id"], "r1");
  EXPECT_EQ(row["local_path"], local);
  EXPECT_EQ(row["remote_path"], "s3://minio-bucket/cam/2026/img.jpg");
  EXPECT_EQ(row["storage_type"], "minio");
  EXPECT_EQ(row["uploaded"], true);
  EXPECT_EQ(row["on_filesystem"], true);
  EXPECT_EQ(row["deleted"], false);
  EXPECT_EQ(row["content_type"], "image/jpeg");
  EXPECT_EQ(row["size"], JPEG_BYTES.size());
  EXPECT_GT(row["updated_at"].get<double>(), 0.0);
  EXPECT_FALSE(row.contains("duration"));

  auto marker = rows_of_kind(*rows, "group_complete")[0];
  EXPECT_EQ(marker["complete"], true);
  EXPECT_EQ(marker["file_count"], 1);
  EXPECT_EQ(marker["files"][0]["local_path"], local);
}

TEST(Uploader, VerifyFailureRetriedWithoutSecondUpload)
{
  Fixture fx({ "minio" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  auto up = fx.uploader(false);
  auto rows = std::make_shared<std::vector<json>>();
  fx.stores[0]->fail_next_heads.store(2);

  auto summary = up.process_record(camera_payload(local, { "minio" }), "dc.measurement.camera", collect_rows(rows));

  EXPECT_EQ(summary.verified, 1u);
  EXPECT_EQ(fx.stores[0]->puts.load(), 1);
  EXPECT_EQ(rows_of_kind(*rows, "file_status").size(), 1u);
}

TEST(Uploader, DeletedOnlyAfterVerifiedOnAllStorages)
{
  Fixture fx({ "minio", "s3_archive" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  auto up = fx.uploader(true);
  auto rows = std::make_shared<std::vector<json>>();
  auto payload = camera_payload(local, { "minio", "s3_archive" });

  fx.stores[1]->fail_put_keys.insert("cam/2026/img.jpg");
  EXPECT_THROW(up.process_record(payload, "dc.measurement.camera", collect_rows(rows)), UploadError);
  EXPECT_TRUE(std::filesystem::exists(local));
  EXPECT_TRUE(rows_of_kind(*rows, "group_complete").empty());

  fx.stores[1]->fail_put_keys.clear();
  auto summary = up.process_record(payload, "dc.measurement.camera", collect_rows(rows));
  EXPECT_EQ(summary.verified, 1u);
  EXPECT_EQ(summary.deleted, 1u);
  EXPECT_TRUE(summary.group_complete);
  EXPECT_FALSE(std::filesystem::exists(local));
  EXPECT_EQ(fx.stores[0]->puts.load(), 1);  // minio uploaded once, not re-uploaded

  auto deleted = rows_of_kind(*rows, "file_status");
  int n_deleted = 0;
  for (auto& r : deleted)
  {
    if (r["deleted"] == true)
    {
      ++n_deleted;
      EXPECT_EQ(r["on_filesystem"], false);
      EXPECT_EQ(r["uploaded"], true);
    }
  }
  EXPECT_EQ(n_deleted, 2);

  auto before = rows->size();
  auto summary2 = up.process_record(payload, "dc.measurement.camera", collect_rows(rows));
  EXPECT_TRUE(summary2.group_complete);
  EXPECT_EQ(summary2.missing, 0u);
  EXPECT_EQ(rows->size(), before);  // no duplicate rows
}

TEST(Uploader, RetriedRecordsDoNotDuplicateRows)
{
  Fixture fx({ "minio" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  auto up = fx.uploader(false);
  auto rows = std::make_shared<std::vector<json>>();
  auto payload = camera_payload(local, { "minio" });

  up.process_record(payload, "dc.measurement.camera", collect_rows(rows));
  auto after_first = rows->size();
  auto summary = up.process_record(payload, "dc.measurement.camera", collect_rows(rows));

  EXPECT_TRUE(summary.group_complete);
  EXPECT_EQ(fx.stores[0]->puts.load(), 1);
  EXPECT_EQ(rows->size(), after_first);
}

TEST(Uploader, MissingFileReportedAndNeverCompletes)
{
  Fixture fx({ "minio" });
  auto local = (fx.tmp / "never-written.jpg").string();
  auto up = fx.uploader(false);
  auto rows = std::make_shared<std::vector<json>>();

  auto summary = up.process_record(camera_payload(local, { "minio" }), "dc.measurement.camera", collect_rows(rows));

  EXPECT_EQ(summary.missing, 1u);
  EXPECT_EQ(summary.verified, 0u);
  EXPECT_FALSE(summary.group_complete);
  auto row = rows_of_kind(*rows, "file_status")[0];
  EXPECT_EQ(row["uploaded"], false);
  EXPECT_EQ(row["on_filesystem"], false);
  EXPECT_TRUE(rows_of_kind(*rows, "group_complete").empty());
  EXPECT_FALSE(fx.stores[0]->has_object("cam/2026/img.jpg"));
}

TEST(Uploader, GroupCompletionOnlyAfterEveryFileVerified)
{
  Fixture fx({ "minio" });
  auto yaml = fx.write_file("map.yaml", "image: map.pgm\nresolution: 0.05\n");
  auto pgm = fx.write_file("map.pgm", std::string("P5\n2 2\n255\n") + std::string("\x00\x01\x02\x03", 4));
  json payload{ { "name", "map" },
                { "local_paths", { { "yaml", yaml }, { "pgm", pgm } } },
                { "remote_paths", { { "minio", { { "yaml", "maps/map.yaml" }, { "pgm", "maps/map.pgm" } } } } } };
  auto up = fx.uploader(false);
  auto rows = std::make_shared<std::vector<json>>();

  fx.stores[0]->fail_put_keys.insert("maps/map.pgm");
  EXPECT_THROW(up.process_record(payload, "dc.measurement.map", collect_rows(rows)), UploadError);
  EXPECT_EQ(rows_of_kind(*rows, "file_status").size(), 1u);
  EXPECT_TRUE(rows_of_kind(*rows, "group_complete").empty());

  fx.stores[0]->fail_put_keys.clear();
  auto summary = up.process_record(payload, "dc.measurement.map", collect_rows(rows));
  EXPECT_TRUE(summary.group_complete);
  EXPECT_EQ(rows_of_kind(*rows, "file_status").size(), 2u);
  auto markers = rows_of_kind(*rows, "group_complete");
  EXPECT_EQ(markers.size(), 1u);
  EXPECT_EQ(markers[0]["file_count"], 2);
}

TEST(Uploader, VideoDurationOnlyForVideoContentTypes)
{
  Fixture fx({ "minio" });
  std::string mp4 = std::string("\x00\x00\x00\x20", 4) + "ftypisom" + std::string(64, '\0');
  auto video = fx.write_file("clip.mp4", mp4);
  auto image = fx.write_file("img.jpg", JPEG_BYTES);

  auto probed = std::make_shared<std::vector<std::string>>();
  UploaderConfig cfg = fx.config(false);
  Uploader up(cfg, fx.storages, [probed](const std::string& path) -> std::optional<double> {
    probed->push_back(path);
    return 12.34;
  });
  json payload{ { "name", "inspection" },
                { "local_paths", { { "video", video }, { "still", image } } },
                { "remote_paths", { { "minio", { { "video", "insp/clip.mp4" }, { "still", "insp/img.jpg" } } } } } };
  auto rows = std::make_shared<std::vector<json>>();
  up.process_record(payload, "dc.measurement.camera", collect_rows(rows));

  auto by_suffix = [&](const std::string& suffix) {
    for (auto& r : rows_of_kind(*rows, "file_status"))
    {
      std::string lp = r["local_path"].get<std::string>();
      if (lp.size() >= suffix.size() && lp.compare(lp.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        return r;
      }
    }
    ADD_FAILURE() << "no row for " << suffix;
    return json{};
  };
  EXPECT_EQ(by_suffix("clip.mp4")["duration"], 12.34);
  EXPECT_EQ(by_suffix("clip.mp4")["content_type"], "video/mp4");
  EXPECT_FALSE(by_suffix("img.jpg").contains("duration"));
  EXPECT_EQ(probed->size(), 1u);  // prober runs only for the video
}

TEST(Uploader, FfprobeProberParsesOutput)
{
  auto tmp = std::filesystem::temp_directory_path() / ("dc_ffprobe_" + std::to_string(::getpid()));
  std::filesystem::create_directories(tmp);
  auto stub = tmp / "ffprobe";
  std::ofstream(stub) << "#!/bin/sh\necho 3.25\n";
  std::filesystem::permissions(stub, std::filesystem::perms::owner_all);

  auto prober = ffprobe_duration_prober(stub.string());
  auto d = prober(tmp.string());
  ASSERT_TRUE(d.has_value());
  EXPECT_DOUBLE_EQ(*d, 3.25);

  auto missing = ffprobe_duration_prober((tmp / "nonexistent").string());
  EXPECT_FALSE(missing(tmp.string()).has_value());
  std::filesystem::remove_all(tmp);
}

TEST(Uploader, InterruptedMultipartResumes)
{
  Fixture fx({ "minio" });
  std::string content(4096, '\0');
  for (std::uint32_t i = 0; i < 4096; ++i)
  {
    content[i] = static_cast<char>(i % 251);
  }
  auto local = fx.write_file("big.bin", content);
  json payload{ { "name", "video_batch" },
                { "local_paths", { { "bin", local } } },
                { "remote_paths", { { "minio", { { "bin", "big/big.bin" } } } } } };

  UploaderConfig cfg = fx.config(false);
  cfg.multipart_threshold_bytes = 1024;
  cfg.multipart_part_size_bytes = 1024;
  cfg.max_attempts = 1;

  fx.stores[0]->allow_parts.store(2);
  {
    Uploader up(cfg, fx.storages, [](const std::string&) { return std::nullopt; });
    auto rows = std::make_shared<std::vector<json>>();
    EXPECT_THROW(up.process_record(payload, "dc.measurement.camera", collect_rows(rows)), UploadError);
  }
  EXPECT_EQ(fx.stores[0]->part_puts, (std::vector<int>{ 0, 1 }));
  EXPECT_FALSE(fx.stores[0]->has_object("big/big.bin"));

  fx.stores[0]->allow_parts.store(std::numeric_limits<long>::max());
  {
    Uploader up(cfg, fx.storages, [](const std::string&) { return std::nullopt; });
    auto rows = std::make_shared<std::vector<json>>();
    auto summary = up.process_record(payload, "dc.measurement.camera", collect_rows(rows));
    EXPECT_EQ(summary.verified, 1u);
    EXPECT_TRUE(summary.group_complete);
  }
  EXPECT_EQ(fx.stores[0]->part_puts, (std::vector<int>{ 0, 1, 2, 3 }));
  EXPECT_EQ(fx.stores[0]->multipart_creates.load(), 1);  // reused, not recreated
  EXPECT_EQ(fx.stores[0]->object_bytes("big/big.bin"), content);
  // resume state cleaned up
  int leftovers = 0;
  for (auto& e : std::filesystem::directory_iterator(fx.tmp / "state"))
  {
    (void)e;
    ++leftovers;
  }
  EXPECT_EQ(leftovers, 0);
}

TEST(Uploader, RecordsWithoutFilesAreNoops)
{
  Fixture fx({ "minio" });
  auto up = fx.uploader(true);
  auto rows = std::make_shared<std::vector<json>>();
  auto summary = up.process_record(json{ { "uptime_s", 42 } }, "dc.measurement.uptime", collect_rows(rows));
  EXPECT_EQ(summary.files, 0u);
  EXPECT_TRUE(rows->empty());
}
