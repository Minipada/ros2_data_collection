// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Exercises the durable intent queue (ADR-0005 follow-up, #265) purely on disk — no ROS,
// no Uploader/ObjectStore needed.
#include "dc_bridge/uploader/intent_queue.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <set>
#include <string>

using namespace dc_bridge::uploader;
using nlohmann::json;

namespace
{

struct Fixture
{
  std::filesystem::path dir;

  Fixture()
  {
    dir = std::filesystem::temp_directory_path() /
          ("dc_intent_queue_test_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    std::filesystem::create_directories(dir);
  }
  ~Fixture()
  {
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
  }

  static std::atomic<int> counter_;
};
std::atomic<int> Fixture::counter_{ 0 };

}  // namespace

TEST(IntentQueue, EnqueueWritesOneJsonFileImmediatelyReady)
{
  Fixture fx;
  IntentQueue q(fx.dir.string());
  EXPECT_TRUE(q.empty());

  const std::string id = q.enqueue("dc.measurement.camera", json{ { "name", "camera" } });

  EXPECT_EQ(q.size(), 1u);
  int json_files = 0;
  for (auto& e : std::filesystem::directory_iterator(fx.dir))
  {
    if (e.path().extension() == ".json")
    {
      ++json_files;
    }
  }
  EXPECT_EQ(json_files, 1);

  auto ready = q.next_ready();
  ASSERT_TRUE(ready.has_value());
  EXPECT_EQ(ready->id, id);
  EXPECT_EQ(ready->tag, "dc.measurement.camera");
  EXPECT_EQ(ready->payload["name"], "camera");
}

TEST(IntentQueue, AckRemovesTheIntentFileAndFromScheduling)
{
  Fixture fx;
  IntentQueue q(fx.dir.string());
  const std::string id = q.enqueue("dc.measurement.map", json{ { "name", "map" } });

  q.ack(id);

  EXPECT_TRUE(q.empty());
  EXPECT_FALSE(q.next_ready().has_value());
  int remaining = 0;
  for (auto& e : std::filesystem::directory_iterator(fx.dir))
  {
    (void)e;
    ++remaining;
  }
  EXPECT_EQ(remaining, 0);
}

TEST(IntentQueue, AckIsIdempotentForAnUnknownId)
{
  Fixture fx;
  IntentQueue q(fx.dir.string());
  EXPECT_NO_THROW(q.ack("00000000000000000000-000000.json"));
  EXPECT_TRUE(q.empty());
}

TEST(IntentQueue, CrashReplayResumesPendingIntentsOldestFirst)
{
  Fixture fx;
  std::string first_id;
  {
    IntentQueue q(fx.dir.string());
    first_id = q.enqueue("dc.measurement.camera", json{ { "seq", 1 } });
    q.enqueue("dc.measurement.camera", json{ { "seq", 2 } });
  }  // "crash" — queue instance destroyed without acking anything

  IntentQueue replayed(fx.dir.string());
  EXPECT_EQ(replayed.size(), 2u);
  auto first = replayed.next_ready();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->id, first_id);
  EXPECT_EQ(first->payload["seq"], 1);
}

TEST(IntentQueue, ReplayIsIdempotentAckingAfterReprocessingDropsNoOthers)
{
  Fixture fx;
  std::string id_a, id_b;
  {
    IntentQueue q(fx.dir.string());
    id_a = q.enqueue("dc.measurement.camera", json{ { "seq", 1 } });
    id_b = q.enqueue("dc.measurement.camera", json{ { "seq", 2 } });
  }

  IntentQueue replayed(fx.dir.string());
  // Simulate re-processing both replayed intents idempotently and acking each.
  auto a = replayed.next_ready();
  ASSERT_TRUE(a.has_value());
  replayed.ack(a->id);
  auto b = replayed.next_ready();
  ASSERT_TRUE(b.has_value());
  replayed.ack(b->id);

  EXPECT_TRUE(replayed.empty());
  EXPECT_EQ(std::set<std::string>({ a->id, b->id }), std::set<std::string>({ id_a, id_b }));
}

TEST(IntentQueue, TmpLeftoversFromACrashMidWriteAreIgnoredOnReplay)
{
  Fixture fx;
  {
    IntentQueue q(fx.dir.string());
    q.enqueue("dc.measurement.camera", json{ { "seq", 1 } });
  }
  // A crash mid tmp+rename leaves a stray .tmp file behind.
  std::ofstream(fx.dir / "99999999999999999999-000001.json.tmp") << "{not even valid json";

  IntentQueue replayed(fx.dir.string());
  EXPECT_EQ(replayed.size(), 1u);  // the stray .tmp is not counted as a pending intent
  auto ready = replayed.next_ready();
  ASSERT_TRUE(ready.has_value());
  EXPECT_EQ(ready->payload["seq"], 1);
}

TEST(IntentQueue, RoundRobinBackoffMakesProgressPastAPermanentlyFailingEntry)
{
  Fixture fx;
  IntentQueue q(fx.dir.string(), std::chrono::milliseconds(1000), std::chrono::milliseconds(60000));
  const std::string bad_id = q.enqueue("dc.measurement.camera", json{ { "seq", "bad" } });
  const std::string good_id = q.enqueue("dc.measurement.camera", json{ { "seq", "good" } });

  auto now = std::chrono::steady_clock::now();

  // First sweep: the bad entry is oldest, so it's picked first; simulate it failing
  // forever by immediately recording another failure every time it's returned.
  auto first = q.next_ready(now);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->id, bad_id);
  q.record_failure(bad_id, now);

  // The bad entry is now backing off; the sweep must not get stuck behind it — the good
  // entry, still immediately ready, must be the next thing returned.
  auto second = q.next_ready(now);
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->id, good_id);
  q.ack(good_id);

  // While still within the backoff window, nothing is ready (the only remaining entry
  // is the backed-off bad one).
  EXPECT_FALSE(q.next_ready(now + std::chrono::milliseconds(500)).has_value());

  // Once the backoff elapses, the bad entry becomes ready again (and can fail forever
  // without ever being dropped — no cap, no dead-letter).
  auto third = q.next_ready(now + std::chrono::milliseconds(1001));
  ASSERT_TRUE(third.has_value());
  EXPECT_EQ(third->id, bad_id);
}

TEST(IntentQueue, BackoffDoublesAndCapsAtMaxBackoff)
{
  Fixture fx;
  IntentQueue q(fx.dir.string(), std::chrono::milliseconds(1000), std::chrono::milliseconds(3000));
  const std::string id = q.enqueue("dc.measurement.camera", json{ { "seq", 1 } });
  auto now = std::chrono::steady_clock::now();

  q.record_failure(id, now);  // backoff -> 1000ms
  EXPECT_FALSE(q.next_ready(now + std::chrono::milliseconds(999)).has_value());
  EXPECT_TRUE(q.next_ready(now + std::chrono::milliseconds(1001)).has_value());

  q.record_failure(id, now + std::chrono::milliseconds(1001));  // backoff -> 2000ms
  EXPECT_FALSE(q.next_ready(now + std::chrono::milliseconds(1001) + std::chrono::milliseconds(1999)).has_value());
  EXPECT_TRUE(q.next_ready(now + std::chrono::milliseconds(1001) + std::chrono::milliseconds(2001)).has_value());

  q.record_failure(id, now);  // would be 4000ms uncapped; must cap at max_backoff (3000ms)
  EXPECT_FALSE(q.next_ready(now + std::chrono::milliseconds(2999)).has_value());
  EXPECT_TRUE(q.next_ready(now + std::chrono::milliseconds(3001)).has_value());
}

TEST(IntentQueue, RescanPicksUpAnIntentEnqueuedByAnotherInstance)
{
  // Models the #446 split: one IntentQueue instance (the Bridge process) enqueues, a
  // second instance (the Uploader process) over the same directory only learns about it
  // via rescan() — enqueue() on the first instance never touches the second's in-memory
  // state.
  Fixture fx;
  IntentQueue writer(fx.dir.string());
  IntentQueue reader(fx.dir.string());
  EXPECT_TRUE(reader.empty());

  const std::string id = writer.enqueue("dc.measurement.camera", json{ { "name", "camera" } });
  EXPECT_TRUE(reader.empty());  // not yet rescanned

  EXPECT_EQ(reader.rescan(), 1u);
  EXPECT_EQ(reader.size(), 1u);
  auto ready = reader.next_ready();
  ASSERT_TRUE(ready.has_value());
  EXPECT_EQ(ready->id, id);
  EXPECT_EQ(ready->payload["name"], "camera");
}

TEST(IntentQueue, RescanLeavesAlreadyKnownEntriesAndTheirBackoffUntouched)
{
  Fixture fx;
  IntentQueue reader(fx.dir.string(), std::chrono::milliseconds(1000), std::chrono::milliseconds(60000));
  const std::string id = reader.enqueue("dc.measurement.camera", json{ { "seq", 1 } });
  const auto now = std::chrono::steady_clock::now();
  reader.record_failure(id, now);  // now backing off

  EXPECT_EQ(reader.rescan(), 0u);                                                     // nothing new on disk
  EXPECT_FALSE(reader.next_ready(now + std::chrono::milliseconds(500)).has_value());  // backoff preserved
}

TEST(IntentQueue, RescanNeverRediscoversAnAlreadyAckedIntent)
{
  Fixture fx;
  IntentQueue writer(fx.dir.string());
  IntentQueue reader(fx.dir.string());
  const std::string id = writer.enqueue("dc.measurement.camera", json{ { "seq", 1 } });

  ASSERT_EQ(reader.rescan(), 1u);
  reader.ack(id);
  EXPECT_TRUE(reader.empty());

  EXPECT_EQ(reader.rescan(), 0u);  // the file is gone; nothing to rediscover
  EXPECT_TRUE(reader.empty());
}

TEST(IntentQueue, RecordsWithoutFilesNeverLingerInSteadyState)
{
  // A files-less Record still gets enqueued (the callback that writes to disk doesn't
  // parse payloads), but a caller that acks immediately after a successful no-op
  // process_record leaves nothing pending — the queue's steady state never grows for
  // these Records, matching #265's "never touches the queue's steady-state" criterion.
  Fixture fx;
  IntentQueue q(fx.dir.string());
  const std::string id = q.enqueue("dc.measurement.uptime", json{ { "uptime_s", 42 } });
  auto ready = q.next_ready();
  ASSERT_TRUE(ready.has_value());
  q.ack(ready->id);

  EXPECT_TRUE(q.empty());
  (void)id;
}
