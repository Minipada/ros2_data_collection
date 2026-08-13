// Exercises the Uploader against an in-memory ObjectStore fake with failure injection —
// no aws-sdk-cpp / cloud dependency.
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
    part_puts.push_back(part_number - 1);  // record 0-based
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

// --- Optional thumbnails (#256) ---------------------------------------------------
//
// Driven through a fake ThumbnailGenerator rather than real ffmpeg: what has to hold is
// the Uploader's *contract* around previews — off unless asked for, strictly after the
// primary upload, never able to fail it, idempotent on replay, no local residue — and
// none of that depends on a decoder actually being installed on the test machine.

namespace
{

/// A ThumbnailGenerator that records every request and writes fixed bytes, or fails.
struct FakeThumbnailer
{
  std::shared_ptr<std::vector<std::string>> sources = std::make_shared<std::vector<std::string>>();
  bool succeed = true;
  std::string bytes = "thumb-bytes";

  ThumbnailGenerator fn()
  {
    auto sources_ = sources;
    auto succeed_ = succeed;
    auto bytes_ = bytes;
    return [sources_, succeed_, bytes_](const std::string& source, const std::string& dest, std::uint32_t) -> bool {
      sources_->push_back(source);
      if (!succeed_)
      {
        return false;
      }
      std::ofstream(dest, std::ios::binary) << bytes_;
      return true;
    };
  }
};

std::size_t count_files(const std::filesystem::path& dir)
{
  std::error_code ec;
  std::size_t n = 0;
  for (auto it = std::filesystem::directory_iterator(dir, ec); !ec && it != std::filesystem::directory_iterator(); ++it)
  {
    ++n;
  }
  return n;
}

}  // namespace

TEST(UploaderThumbnails, OffByDefault)
{
  Fixture fx({ "minio" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  FakeThumbnailer thumbs;
  // Config left at its defaults — thumbnails.enabled is false — even though a generator
  // is wired in, so "off by default" is a property of the config, not of the plumbing.
  Uploader up(
      fx.config(false), fx.storages, [](const std::string&) { return std::nullopt; }, thumbs.fn());
  auto rows = std::make_shared<std::vector<json>>();

  auto summary = up.process_record(camera_payload(local, { "minio" }), "dc.measurement.camera", collect_rows(rows));

  EXPECT_TRUE(thumbs.sources->empty());
  EXPECT_EQ(summary.thumbnails, 0u);
  EXPECT_EQ(summary.thumbnails_failed, 0u);
  EXPECT_FALSE(fx.stores[0]->has_object("cam/2026/img.jpg.thumb.jpg"));
  EXPECT_FALSE(rows_of_kind(*rows, "file_status")[0].contains("thumbnail_path"));
}

TEST(UploaderThumbnails, UploadsPreviewNextToTheOriginalAndRecordsItsPath)
{
  Fixture fx({ "minio" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  FakeThumbnailer thumbs;
  UploaderConfig cfg = fx.config(false);
  cfg.thumbnails.enabled = true;
  Uploader up(
      cfg, fx.storages, [](const std::string&) { return std::nullopt; }, thumbs.fn());
  auto rows = std::make_shared<std::vector<json>>();

  auto summary = up.process_record(camera_payload(local, { "minio" }), "dc.measurement.camera", collect_rows(rows));

  EXPECT_EQ(summary.thumbnails, 1u);
  EXPECT_EQ(summary.thumbnails_failed, 0u);
  ASSERT_EQ(thumbs.sources->size(), 1u);
  EXPECT_EQ((*thumbs.sources)[0], local);  // derived from the original, not from the object store
  EXPECT_EQ(fx.stores[0]->object_bytes("cam/2026/img.jpg.thumb.jpg"), "thumb-bytes");

  auto row = rows_of_kind(*rows, "file_status")[0];
  ASSERT_TRUE(row.contains("thumbnail_path"));
  EXPECT_EQ(row["thumbnail_path"], "s3://minio-bucket/cam/2026/img.jpg.thumb.jpg");
  // The preview decorates the File's own row; it is not a File in its own right, so it
  // gets no separate status row and doesn't inflate the group.
  EXPECT_EQ(rows_of_kind(*rows, "file_status").size(), 1u);
  EXPECT_EQ(rows_of_kind(*rows, "group_complete")[0]["file_count"], 1);
}

TEST(UploaderThumbnails, GenerationFailureNeverBlocksOrFailsThePrimaryUpload)
{
  Fixture fx({ "minio" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  FakeThumbnailer thumbs;
  thumbs.succeed = false;  // e.g. no ffmpeg on PATH, or an undecodable File.
  UploaderConfig cfg = fx.config(false);
  cfg.thumbnails.enabled = true;
  Uploader up(
      cfg, fx.storages, [](const std::string&) { return std::nullopt; }, thumbs.fn());
  auto rows = std::make_shared<std::vector<json>>();

  // No throw: a failed preview is not an incomplete upload, so the intent is acked and
  // never retried on the preview's account.
  ProcessSummary summary;
  ASSERT_NO_THROW(
      summary = up.process_record(camera_payload(local, { "minio" }), "dc.measurement.camera", collect_rows(rows)));

  EXPECT_EQ(summary.verified, 1u);
  EXPECT_TRUE(summary.group_complete);
  EXPECT_EQ(summary.thumbnails, 0u);
  EXPECT_EQ(summary.thumbnails_failed, 1u);  // counted, so the operator can see it
  EXPECT_EQ(fx.stores[0]->object_bytes("cam/2026/img.jpg"), JPEG_BYTES);
  EXPECT_FALSE(rows_of_kind(*rows, "file_status")[0].contains("thumbnail_path"));
}

TEST(UploaderThumbnails, StoreRejectingThePreviewLeavesTheUploadVerified)
{
  Fixture fx({ "minio" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  FakeThumbnailer thumbs;
  UploaderConfig cfg = fx.config(false);
  cfg.thumbnails.enabled = true;
  Uploader up(
      cfg, fx.storages, [](const std::string&) { return std::nullopt; }, thumbs.fn());
  auto rows = std::make_shared<std::vector<json>>();
  fx.stores[0]->fail_put_keys.insert("cam/2026/img.jpg.thumb.jpg");

  ProcessSummary summary;
  ASSERT_NO_THROW(
      summary = up.process_record(camera_payload(local, { "minio" }), "dc.measurement.camera", collect_rows(rows)));

  EXPECT_EQ(summary.verified, 1u);
  EXPECT_TRUE(summary.group_complete);
  EXPECT_EQ(summary.thumbnails_failed, 1u);
  EXPECT_FALSE(rows_of_kind(*rows, "file_status")[0].contains("thumbnail_path"));
}

TEST(UploaderThumbnails, OnlyForImagesAndVideos)
{
  Fixture fx({ "minio" });
  auto yaml = fx.write_file("map.yaml", "image: map.pgm\nresolution: 0.05\n");
  auto pgm = fx.write_file("map.pgm", std::string("P5\n2 2\n255\n") + std::string("\x00\x01\x02\x03", 4));
  json payload{ { "name", "map" },
                { "local_paths", { { "yaml", yaml }, { "pgm", pgm } } },
                { "remote_paths", { { "minio", { { "yaml", "maps/map.yaml" }, { "pgm", "maps/map.pgm" } } } } } };
  FakeThumbnailer thumbs;
  UploaderConfig cfg = fx.config(false);
  cfg.thumbnails.enabled = true;
  Uploader up(
      cfg, fx.storages, [](const std::string&) { return std::nullopt; }, thumbs.fn());
  auto rows = std::make_shared<std::vector<json>>();

  auto summary = up.process_record(payload, "dc.measurement.map", collect_rows(rows));

  // The PGM gets one; the YAML sidecar is never handed to a decoder at all, and so is
  // not counted as a failure either.
  ASSERT_EQ(thumbs.sources->size(), 1u);
  EXPECT_EQ((*thumbs.sources)[0], pgm);
  EXPECT_EQ(summary.thumbnails, 1u);
  EXPECT_EQ(summary.thumbnails_failed, 0u);
  EXPECT_TRUE(fx.stores[0]->has_object("maps/map.pgm.thumb.jpg"));
  EXPECT_FALSE(fx.stores[0]->has_object("maps/map.yaml.thumb.jpg"));
}

TEST(UploaderThumbnails, ReplayedIntentReusesThePreviewInsteadOfRegeneratingIt)
{
  Fixture fx({ "minio" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  FakeThumbnailer thumbs;
  UploaderConfig cfg = fx.config(false);
  cfg.thumbnails.enabled = true;
  auto payload = camera_payload(local, { "minio" });

  // Two Uploader instances, as a Bridge restart produces (#265): the second replays the
  // same intent with an empty in-memory dedup set, so only the object store's own state
  // can prevent redundant work.
  {
    Uploader up(
        cfg, fx.storages, [](const std::string&) { return std::nullopt; }, thumbs.fn());
    auto rows = std::make_shared<std::vector<json>>();
    up.process_record(payload, "dc.measurement.camera", collect_rows(rows));
  }
  {
    Uploader up(
        cfg, fx.storages, [](const std::string&) { return std::nullopt; }, thumbs.fn());
    auto rows = std::make_shared<std::vector<json>>();
    auto summary = up.process_record(payload, "dc.measurement.camera", collect_rows(rows));
    EXPECT_EQ(summary.thumbnails, 1u);
    // Still reported on the replayed row, so a status Record re-emitted after a restart
    // isn't missing the preview the first one carried.
    EXPECT_EQ(rows_of_kind(*rows, "file_status")[0]["thumbnail_path"], "s3://minio-bucket/cam/2026/img.jpg.thumb.jpg");
  }
  EXPECT_EQ(thumbs.sources->size(), 1u);  // decoded once across both runs
}

TEST(UploaderThumbnails, EveryDestinationGetsItsOwnPreview)
{
  Fixture fx({ "minio", "s3_archive" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  FakeThumbnailer thumbs;
  UploaderConfig cfg = fx.config(false);
  cfg.thumbnails.enabled = true;
  Uploader up(
      cfg, fx.storages, [](const std::string&) { return std::nullopt; }, thumbs.fn());
  auto rows = std::make_shared<std::vector<json>>();

  auto summary =
      up.process_record(camera_payload(local, { "minio", "s3_archive" }), "dc.measurement.camera", collect_rows(rows));

  EXPECT_EQ(summary.thumbnails, 2u);
  for (auto& store : fx.stores)
  {
    EXPECT_EQ(store->object_bytes("cam/2026/img.jpg.thumb.jpg"), "thumb-bytes");
  }
}

TEST(UploaderThumbnails, LeaveNoLocalResidue)
{
  // A preview is scratch, not collected data: it must not linger next to the original
  // (where delete_when_sent would never remove it) nor in the state dir (where it would
  // grow without bound). Its lifetime being contained inside one process_record is also
  // what makes "a thumbnail never outlives its original" true by construction for
  // retention (#267) — a shed intent's Files never uploaded, so none was ever derived.
  Fixture fx({ "minio" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  FakeThumbnailer thumbs;
  UploaderConfig cfg = fx.config(true);  // delete_when_sent
  cfg.thumbnails.enabled = true;
  Uploader up(
      cfg, fx.storages, [](const std::string&) { return std::nullopt; }, thumbs.fn());
  auto rows = std::make_shared<std::vector<json>>();

  auto summary = up.process_record(camera_payload(local, { "minio" }), "dc.measurement.camera", collect_rows(rows));

  EXPECT_EQ(summary.thumbnails, 1u);
  EXPECT_EQ(summary.deleted, 1u);
  EXPECT_FALSE(std::filesystem::exists(local));
  EXPECT_EQ(count_files(fx.tmp / "state" / "thumbs"), 0u);
  // Nothing was written beside the original either.
  EXPECT_FALSE(std::filesystem::exists(local + ".thumb.jpg"));
}

TEST(UploaderThumbnails, AFileThatNeverUploadsNeverGetsAPreview)
{
  // The other half of "a preview obeys its original's retention policy" (#267): because
  // generation is gated on the File being verified, an intent that never uploads — the
  // only kind retention ever sheds — cannot leave an orphaned preview on the store
  // pointing at data that was abandoned.
  Fixture fx({ "minio" });
  auto local = fx.write_file("img.jpg", JPEG_BYTES);
  FakeThumbnailer thumbs;
  UploaderConfig cfg = fx.config(false);
  cfg.thumbnails.enabled = true;
  cfg.max_attempts = 1;
  Uploader up(
      cfg, fx.storages, [](const std::string&) { return std::nullopt; }, thumbs.fn());
  auto rows = std::make_shared<std::vector<json>>();
  fx.stores[0]->fail_put_keys.insert("cam/2026/img.jpg");

  EXPECT_THROW(up.process_record(camera_payload(local, { "minio" }), "dc.measurement.camera", collect_rows(rows)),
               UploadError);

  EXPECT_TRUE(thumbs.sources->empty());
  EXPECT_FALSE(fx.stores[0]->has_object("cam/2026/img.jpg.thumb.jpg"));
  EXPECT_EQ(count_files(fx.tmp / "state" / "thumbs"), 0u);
}

TEST(UploaderThumbnails, MissingLocalFileNeverAttemptsAPreview)
{
  Fixture fx({ "minio" });
  auto local = (fx.tmp / "never-written.jpg").string();
  FakeThumbnailer thumbs;
  UploaderConfig cfg = fx.config(false);
  cfg.thumbnails.enabled = true;
  Uploader up(
      cfg, fx.storages, [](const std::string&) { return std::nullopt; }, thumbs.fn());
  auto rows = std::make_shared<std::vector<json>>();

  auto summary = up.process_record(camera_payload(local, { "minio" }), "dc.measurement.camera", collect_rows(rows));

  EXPECT_EQ(summary.missing, 1u);
  EXPECT_TRUE(thumbs.sources->empty());
  EXPECT_EQ(summary.thumbnails_failed, 0u);
}
