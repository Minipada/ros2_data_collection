// Exercises the retention sweep (#267) purely on disk with an injected
// VerifiedEverywhereFn — no aws-sdk-cpp / cloud dependency, no real ObjectStore needed
// (sweep() itself never calls Storage::store; only object_key()/name/url_prefix).
#include "dc_bridge/uploader/retention.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using namespace dc_bridge;
using namespace dc_bridge::uploader;
using nlohmann::json;

namespace
{

struct Fixture
{
  std::filesystem::path tmp;
  std::filesystem::path queue_dir;

  Fixture()
  {
    tmp = std::filesystem::temp_directory_path() /
          ("dc_retention_test_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    queue_dir = tmp / "queue";
    std::filesystem::create_directories(tmp);
  }
  ~Fixture()
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp, ec);
  }

  std::string write_file(const std::string& name, std::size_t bytes)
  {
    auto p = tmp / name;
    std::ofstream(p, std::ios::binary) << std::string(bytes, 'x');
    return p.string();
  }

  static std::atomic<int> counter_;
};
std::atomic<int> Fixture::counter_{ 0 };

json camera_payload(const std::string& group_name, const std::string& local_path, const std::string& storage_name)
{
  return json{ { "name", group_name },
               { "local_paths", { { "raw", local_path } } },
               { "remote_paths", { { storage_name, { { "raw", "cam/" + group_name + ".jpg" } } } } } };
}

Storage make_storage(const std::string& name)
{
  return Storage{ name, "s3://" + name + "-bucket/", "", nullptr };
}

VerifiedEverywhereFn never_verified()
{
  return [](const FileGroup&) { return false; };
}

}  // namespace

TEST(Retention, SweepIsNoopWhenDisabled)
{
  Fixture fx;
  IntentQueue q(fx.queue_dir.string());
  auto local = fx.write_file("img.jpg", 1000);
  q.enqueue("dc.measurement.camera", camera_payload("g1", local, "minio"));

  RetentionConfig cfg;  // default: disabled
  auto rows = std::make_shared<std::vector<json>>();
  auto warns = std::make_shared<std::vector<std::string>>();
  auto result = sweep(
      q, { make_storage("minio") }, cfg, never_verified(), [rows](const json& r) { rows->push_back(r); },
      [warns](const std::string& w) { warns->push_back(w); });

  EXPECT_EQ(result.shed_intents, 0u);
  EXPECT_EQ(result.bytes_freed, 0u);
  EXPECT_EQ(q.size(), 1u);
  EXPECT_TRUE(std::filesystem::exists(local));
  EXPECT_TRUE(rows->empty());
  EXPECT_TRUE(warns->empty());
}

TEST(Retention, ShedsOldestFirstUntilUnderMaxBytes)
{
  Fixture fx;
  IntentQueue q(fx.queue_dir.string());
  auto oldest = fx.write_file("oldest.jpg", 100);
  auto middle = fx.write_file("middle.jpg", 100);
  auto newest = fx.write_file("newest.jpg", 100);
  q.enqueue("dc.measurement.camera", camera_payload("oldest", oldest, "minio"));
  q.enqueue("dc.measurement.camera", camera_payload("middle", middle, "minio"));
  q.enqueue("dc.measurement.camera", camera_payload("newest", newest, "minio"));

  RetentionConfig cfg;
  cfg.max_bytes = 150;  // 300 bytes total pending; must shed down to <=150.
  auto rows = std::make_shared<std::vector<json>>();
  auto result = sweep(
      q, { make_storage("minio") }, cfg, never_verified(), [rows](const json& r) { rows->push_back(r); },
      [](const std::string&) {});

  EXPECT_EQ(result.shed_intents, 2u);
  EXPECT_EQ(result.bytes_freed, 200u);
  EXPECT_EQ(q.size(), 1u);
  EXPECT_FALSE(std::filesystem::exists(oldest));
  EXPECT_FALSE(std::filesystem::exists(middle));
  EXPECT_TRUE(std::filesystem::exists(newest));  // the newest survives — oldest-first.
  EXPECT_EQ(rows->size(), 2u);
}

TEST(Retention, ShedsByMaxAgeEvenUnderBytesLimit)
{
  Fixture fx;
  IntentQueue q(fx.queue_dir.string());
  auto local = fx.write_file("img.jpg", 10);
  q.enqueue("dc.measurement.camera", camera_payload("g1", local, "minio"));
  const auto enqueued_at = q.pending().at(0).enqueued_at;

  RetentionConfig cfg;
  cfg.max_age_days = 30;  // max_bytes left at 0/unlimited — age alone must trigger the shed.
  auto rows = std::make_shared<std::vector<json>>();
  const auto far_future = enqueued_at + std::chrono::hours(24) * 31;
  auto result = sweep(
      q, { make_storage("minio") }, cfg, never_verified(), [rows](const json& r) { rows->push_back(r); },
      [](const std::string&) {}, far_future);

  EXPECT_EQ(result.shed_intents, 1u);
  EXPECT_TRUE(q.empty());
  EXPECT_FALSE(std::filesystem::exists(local));
}

TEST(Retention, WithinMaxAgeIsNotShed)
{
  Fixture fx;
  IntentQueue q(fx.queue_dir.string());
  auto local = fx.write_file("img.jpg", 10);
  q.enqueue("dc.measurement.camera", camera_payload("g1", local, "minio"));
  const auto enqueued_at = q.pending().at(0).enqueued_at;

  RetentionConfig cfg;
  cfg.max_age_days = 30;
  const auto still_fresh = enqueued_at + std::chrono::hours(24) * 5;
  auto result = sweep(
      q, { make_storage("minio") }, cfg, never_verified(), [](const json&) {}, [](const std::string&) {}, still_fresh);

  EXPECT_EQ(result.shed_intents, 0u);
  EXPECT_EQ(q.size(), 1u);
  EXPECT_TRUE(std::filesystem::exists(local));
}

TEST(Retention, VerifiedEverywhereIntentsAreNeverShed)
{
  Fixture fx;
  IntentQueue q(fx.queue_dir.string());
  auto verified_file = fx.write_file("verified.jpg", 200);
  auto unverified_file = fx.write_file("unverified.jpg", 200);
  q.enqueue("dc.measurement.camera", camera_payload("verified", verified_file, "minio"));
  q.enqueue("dc.measurement.camera", camera_payload("unverified", unverified_file, "minio"));

  RetentionConfig cfg;
  cfg.max_bytes = 100;  // both together are well over the limit.
  VerifiedEverywhereFn is_verified = [](const FileGroup& g) { return g.group_name == "verified"; };
  auto result = sweep(
      q, { make_storage("minio") }, cfg, is_verified, [](const json&) {}, [](const std::string&) {});

  // The verified-but-still-pending intent (delete_when_sent's own retry, not retention's
  // job) is skipped; the unverified one — the one actually eligible — is shed instead.
  EXPECT_EQ(result.shed_intents, 1u);
  EXPECT_TRUE(std::filesystem::exists(verified_file));
  EXPECT_FALSE(std::filesystem::exists(unverified_file));
  EXPECT_EQ(q.size(), 1u);
  auto remaining = q.pending();
  ASSERT_EQ(remaining.size(), 1u);
  EXPECT_EQ(remaining[0].payload["name"], "verified");
}

TEST(Retention, IfEveryCandidateIsVerifiedNothingIsShedAndTheLoopTerminates)
{
  Fixture fx;
  IntentQueue q(fx.queue_dir.string());
  auto local = fx.write_file("img.jpg", 1000);
  q.enqueue("dc.measurement.camera", camera_payload("g1", local, "minio"));

  RetentionConfig cfg;
  cfg.max_bytes = 10;
  VerifiedEverywhereFn always_verified = [](const FileGroup&) { return true; };
  auto result = sweep(
      q, { make_storage("minio") }, cfg, always_verified, [](const json&) {}, [](const std::string&) {});

  EXPECT_EQ(result.shed_intents, 0u);
  EXPECT_EQ(q.size(), 1u);
  EXPECT_TRUE(std::filesystem::exists(local));
}

TEST(Retention, FileAndIntentAreAlwaysRemovedTogether)
{
  Fixture fx;
  IntentQueue q(fx.queue_dir.string());
  auto local = fx.write_file("img.jpg", 1000);
  const std::string id = q.enqueue("dc.measurement.camera", camera_payload("g1", local, "minio"));

  RetentionConfig cfg;
  cfg.max_bytes = 1;
  sweep(
      q, { make_storage("minio") }, cfg, never_verified(), [](const json&) {}, [](const std::string&) {});

  EXPECT_FALSE(std::filesystem::exists(local));
  EXPECT_TRUE(q.empty());
  // The intent's own JSON file is gone too (ack unlinks it) — never one without the other.
  EXPECT_FALSE(std::filesystem::exists(fx.queue_dir / id));
}

TEST(Retention, AuditRowHasShedWithoutUploadSignature)
{
  Fixture fx;
  IntentQueue q(fx.queue_dir.string());
  auto local = fx.write_file("img.jpg", 1000);
  q.enqueue("dc.measurement.camera", camera_payload("g1", local, "minio"));

  RetentionConfig cfg;
  cfg.max_bytes = 1;
  auto rows = std::make_shared<std::vector<json>>();
  sweep(
      q, { make_storage("minio") }, cfg, never_verified(), [rows](const json& r) { rows->push_back(r); },
      [](const std::string&) {});

  ASSERT_EQ(rows->size(), 1u);
  const auto& row = rows->at(0);
  EXPECT_EQ(row["kind"], "file_status");
  EXPECT_EQ(row["deleted"], true);
  EXPECT_EQ(row["uploaded"], false);
  EXPECT_EQ(row["on_filesystem"], false);
  EXPECT_EQ(row["storage_type"], "minio");
  EXPECT_EQ(row["local_path"], local);
}

TEST(Retention, EmitFailureDoesNotBlockSheddingButWarnStillFires)
{
  Fixture fx;
  IntentQueue q(fx.queue_dir.string());
  auto local = fx.write_file("img.jpg", 1000);
  q.enqueue("dc.measurement.camera", camera_payload("g1", local, "minio"));

  RetentionConfig cfg;
  cfg.max_bytes = 1;
  auto warns = std::make_shared<std::vector<std::string>>();
  auto result = sweep(
      q, { make_storage("minio") }, cfg, never_verified(),
      [](const json&) { throw std::runtime_error("shipper unreachable"); },
      [warns](const std::string& w) { warns->push_back(w); });

  EXPECT_EQ(result.shed_intents, 1u);
  EXPECT_FALSE(std::filesystem::exists(local));
  EXPECT_TRUE(q.empty());
  ASSERT_EQ(warns->size(), 1u);
  EXPECT_NE(warns->at(0).find("shed 1 intent"), std::string::npos);
}

TEST(Retention, RecordsWithoutFilesAreIgnoredByRetention)
{
  Fixture fx;
  IntentQueue q(fx.queue_dir.string());
  q.enqueue("dc.measurement.uptime", json{ { "uptime_s", 42 } });

  RetentionConfig cfg;
  cfg.max_bytes = 1;
  auto result = sweep(
      q, { make_storage("minio") }, cfg, never_verified(), [](const json&) {}, [](const std::string&) {});

  EXPECT_EQ(result.shed_intents, 0u);
  EXPECT_EQ(q.size(), 1u);  // a files-less Record isn't retention's concern.
}

TEST(Retention, ReplayToleratesAnAlreadyMissingLocalFile)
{
  // Simulates a crash between the file delete and the intent ack: the File is already
  // gone from disk, but the intent is still pending. A later sweep (replay) must still
  // converge cleanly — no throw, no hang, no stranding — rather than getting stuck. Uses
  // max_age_days (not max_bytes) to trigger the shed: an already-missing File
  // legitimately contributes 0 bytes to the pool, so byte-based eviction has nothing left
  // to react to for this one — age is what still lets a leftover pending intent resolve.
  Fixture fx;
  IntentQueue q(fx.queue_dir.string());
  auto local = fx.write_file("img.jpg", 1000);
  q.enqueue("dc.measurement.camera", camera_payload("g1", local, "minio"));
  const auto enqueued_at = q.pending().at(0).enqueued_at;
  std::filesystem::remove(local);  // "crash" already deleted the File; the intent lingers.

  RetentionConfig cfg;
  cfg.max_age_days = 30;
  auto rows = std::make_shared<std::vector<json>>();
  auto result = sweep(
      q, { make_storage("minio") }, cfg, never_verified(), [rows](const json& r) { rows->push_back(r); },
      [](const std::string&) {}, enqueued_at + std::chrono::hours(24) * 31);

  EXPECT_EQ(result.shed_intents, 1u);
  EXPECT_TRUE(q.empty());
  EXPECT_EQ(rows->size(), 1u);  // the audit row still lands even though there was nothing left to unlink.
}
