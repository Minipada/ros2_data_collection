// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "dc_measurements/source_adapter.hpp"

using dc_measurements::SourceAdapter;

// An event source drains one value per poll, in arrival order.
TEST(SourceAdapterTest, PopDrainsOneValuePerCallInArrivalOrder)
{
  SourceAdapter<int> queue{ 8 };
  EXPECT_FALSE(queue.push(1));
  EXPECT_FALSE(queue.push(2));

  const auto first = queue.pop();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(*first, 1);
  const auto second = queue.pop();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(*second, 2);

  EXPECT_FALSE(queue.pop().has_value());
}

// Capacity 1 is the keep-only-the-latest shape: every push displaces the previous value.
TEST(SourceAdapterTest, CapacityOneKeepsOnlyTheNewestValue)
{
  SourceAdapter<int> latest{ 1 };
  EXPECT_FALSE(latest.push(1));
  EXPECT_TRUE(latest.push(2));
  EXPECT_TRUE(latest.push(3));

  const auto value = latest.pop();
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(*value, 3);
  EXPECT_FALSE(latest.pop().has_value());
}

// Past capacity the OLDEST entry goes, and the caller is told so it can warn.
TEST(SourceAdapterTest, OverflowDropsTheOldestAndReportsIt)
{
  SourceAdapter<int> queue{ 2 };
  EXPECT_FALSE(queue.push(1));
  EXPECT_FALSE(queue.push(2));
  EXPECT_TRUE(queue.push(3));

  const auto first = queue.pop();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(*first, 2);
  const auto second = queue.pop();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(*second, 3);
}

// A sample source re-reads the newest value on every poll without draining it.
TEST(SourceAdapterTest, LatestCopiesTheNewestValueWithoutDraining)
{
  SourceAdapter<int> sample{ 1 };
  EXPECT_FALSE(sample.latest().has_value());

  sample.push(1);
  EXPECT_EQ(*sample.latest(), 1);
  EXPECT_EQ(*sample.latest(), 1);
  sample.push(2);
  EXPECT_EQ(*sample.latest(), 2);

  // latest() leaves the value pending for pop().
  const auto value = sample.pop();
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(*value, 2);
  EXPECT_FALSE(sample.latest().has_value());
}

// The handoff the adapter exists for: sources and the poll drain running on different threads
// at full speed. Whatever survives the bounded backlog, each producer's values must come out
// exactly once and in order -- anything else is a lost or duplicated Record.
TEST(SourceAdapterTest, ConcurrentHandoffKeepsPerProducerOrdering)
{
  constexpr int kProducers = 4;
  constexpr int kPerProducer = 4000;
  SourceAdapter<std::pair<int, std::uint64_t>> queue{ 8 };

  std::atomic<bool> producing{ true };
  std::mutex popped_mutex;
  std::vector<std::pair<int, std::uint64_t>> popped;

  std::thread consumer([&] {
    for (;;)
    {
      if (auto value = queue.pop())
      {
        const std::lock_guard<std::mutex> lock(popped_mutex);
        popped.push_back(*value);
      }
      else if (!producing)
      {
        break;
      }
      else
      {
        std::this_thread::yield();
      }
    }
  });
  std::thread sample_reader([&] {
    while (producing)
    {
      (void)queue.latest();
    }
  });

  std::vector<std::thread> producers;
  for (int producer = 0; producer < kProducers; ++producer)
  {
    producers.emplace_back([&queue, producer] {
      for (std::uint64_t i = 0; i < kPerProducer; ++i)
      {
        queue.push({ producer, i });
      }
    });
  }
  for (auto& producer : producers)
  {
    producer.join();
  }
  producing = false;
  consumer.join();
  sample_reader.join();

  // The final backlog (at least `capacity` values) is always drained once producing stops.
  EXPECT_GE(popped.size(), 8u);
  // Every value that survived arrives exactly once, per producer in order. Some were dropped
  // by the bounded backlog -- which sequence numbers survived is not the test.
  std::map<int, std::uint64_t> last_seen;
  for (const auto& [producer, seq] : popped)
  {
    const auto [it, first_seen] = last_seen.emplace(producer, seq);
    if (!first_seen)
    {
      EXPECT_GT(seq, it->second) << "producer " << producer << ": value " << seq << " after " << it->second;
      it->second = seq;
    }
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
