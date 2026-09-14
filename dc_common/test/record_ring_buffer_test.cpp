// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Unit tests for dc_common::RecordRingBuffer (#285, part of #282's pre-event circular-buffer
// capture). No node, no rclcpp::init(): the buffer is exercised purely against explicit
// std::chrono::system_clock::time_point values standing in for an injected/fake clock -- push()
// and evict() both take the timestamp/"now" as a parameter rather than reading a real clock.

#include "dc_common/record_ring_buffer.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <vector>

using dc_common::RecordRingBuffer;
using dc_common::StampedRecord;
using json = nlohmann::json;

namespace
{

std::chrono::system_clock::time_point at(int seconds)
{
  return std::chrono::system_clock::time_point(std::chrono::seconds(seconds));
}

json record(const char* literal)
{
  return json::parse(literal);
}

std::vector<json> records_of(const std::vector<StampedRecord>& entries)
{
  std::vector<json> out;
  out.reserve(entries.size());
  for (const auto& e : entries)
  {
    out.push_back(e.json);
  }
  return out;
}

}  // namespace

TEST(RecordRingBuffer, StartsEmpty)
{
  RecordRingBuffer buf(std::chrono::seconds(10));
  EXPECT_TRUE(buf.empty());
  EXPECT_EQ(buf.size(), 0u);
  EXPECT_TRUE(buf.window().empty());
}

TEST(RecordRingBuffer, PushDoesNotEvictOnItsOwn)
{
  // The window is relative to a caller-supplied "now", not to the last push -- pushing a Record
  // far in the future of an earlier one must not silently drop the earlier one.
  RecordRingBuffer buf(std::chrono::seconds(1));
  buf.push(record(R"({"n":1})"), at(0));
  buf.push(record(R"({"n":2})"), at(100));

  EXPECT_EQ(buf.size(), 2u);
  EXPECT_EQ(records_of(buf.window()), std::vector<json>({ record(R"({"n":1})"), record(R"({"n":2})") }));
}

TEST(RecordRingBuffer, WindowReturnsEntriesInTimeOrderRegardlessOfPushOrder)
{
  RecordRingBuffer buf(std::chrono::seconds(100));
  buf.push(record(R"({"n":"middle"})"), at(5));
  buf.push(record(R"({"n":"first"})"), at(1));
  buf.push(record(R"({"n":"last"})"), at(9));

  EXPECT_EQ(records_of(buf.window()),
            std::vector<json>({ record(R"({"n":"first"})"), record(R"({"n":"middle"})"), record(R"({"n":"last"})") }));
}

TEST(RecordRingBuffer, EvictDropsEntriesStrictlyOlderThanWindow)
{
  RecordRingBuffer buf(std::chrono::seconds(10));
  buf.push(record(R"({"n":"old"})"), at(0));
  buf.push(record(R"({"n":"new"})"), at(5));

  buf.evict(at(11));  // "old" is 11s old (>10s window); "new" is 6s old (<=10s window).

  EXPECT_EQ(buf.size(), 1u);
  EXPECT_EQ(records_of(buf.window()), std::vector<json>({ record(R"({"n":"new"})") }));
}

TEST(RecordRingBuffer, EvictBoundaryAgeEqualToWindowIsKept)
{
  RecordRingBuffer buf(std::chrono::seconds(10));
  buf.push(record(R"({"n":"boundary"})"), at(0));

  buf.evict(at(10));  // age == window exactly: inclusive boundary, must survive.

  EXPECT_EQ(buf.size(), 1u);
}

TEST(RecordRingBuffer, EvictBoundaryOneTickPastWindowIsDropped)
{
  RecordRingBuffer buf(std::chrono::seconds(10));
  buf.push(record(R"({"n":"boundary"})"), at(0));

  buf.evict(at(11));  // age > window by the smallest margin used here: must be dropped.

  EXPECT_TRUE(buf.empty());
}

TEST(RecordRingBuffer, EvictOnEmptyBufferIsANoop)
{
  RecordRingBuffer buf(std::chrono::seconds(10));
  buf.evict(at(1000));
  EXPECT_TRUE(buf.empty());
}

TEST(RecordRingBuffer, EvictLeavesNewerEntriesAfterDroppingOlderOnes)
{
  RecordRingBuffer buf(std::chrono::seconds(10));
  buf.push(record(R"({"n":"a"})"), at(0));
  buf.push(record(R"({"n":"b"})"), at(3));
  buf.push(record(R"({"n":"c"})"), at(6));
  buf.push(record(R"({"n":"d"})"), at(9));

  buf.evict(at(15));  // window is (5, 15]: only entries with stamp > 5 survive -- b(3) and a(0) drop, c(6)/d(9) stay.

  EXPECT_EQ(records_of(buf.window()), std::vector<json>({ record(R"({"n":"c"})"), record(R"({"n":"d"})") }));
}

TEST(RecordRingBuffer, WindowDoesNotMutateOrEvict)
{
  RecordRingBuffer buf(std::chrono::seconds(1));
  buf.push(record(R"({"n":"a"})"), at(0));

  auto first_read = buf.window();
  auto second_read = buf.window();

  EXPECT_EQ(first_read.size(), 1u);
  EXPECT_EQ(second_read.size(), 1u);
  EXPECT_EQ(buf.size(), 1u);  // reading window() twice, well past the window, still hasn't evicted.
}

TEST(RecordRingBuffer, RepeatedEvictCallsConverge)
{
  RecordRingBuffer buf(std::chrono::seconds(5));
  buf.push(record(R"({"n":"a"})"), at(0));

  buf.evict(at(100));
  buf.evict(at(200));  // calling evict again on an already-empty buffer must not throw or hang.

  EXPECT_TRUE(buf.empty());
}

TEST(RecordRingBuffer, ClearDropsEveryEntryRegardlessOfAge)
{
  RecordRingBuffer buf(std::chrono::seconds(10));
  buf.push(record(R"({"n":"a"})"), at(0));
  buf.push(record(R"({"n":"b"})"), at(1));

  buf.clear();  // a consumer that has just released the whole window must not see it again.

  EXPECT_TRUE(buf.empty());
  EXPECT_EQ(buf.size(), 0u);
  EXPECT_TRUE(buf.window().empty());

  buf.push(record(R"({"n":"c"})"), at(2));  // and the buffer is still usable afterwards.
  EXPECT_EQ(records_of(buf.window()), std::vector<json>({ record(R"({"n":"c"})") }));
}
