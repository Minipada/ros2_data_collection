// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Raw / generic-subscription mode against real message types (#227).
//
// The point of every test here is that the Bridge is handed *bytes and a type name* and
// nothing else: `dc_interfaces/msg/StringStamped` is a custom, non-std_msgs type, and
// the converter resolves it through the runtime type support libraries exactly as it
// would for a message package it has never seen. The manager tests additionally run a
// real ROS graph — publisher, discovery, generic subscription — and cover the firehose
// behaviour (#227's "a raw-subscribe-everything mode can outrun the Shipper").
#include "dc_bridge/raw_subscriptions.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <vector>

#include "dc_interfaces/msg/string_stamped.hpp"

using namespace dc_bridge;
using namespace std::chrono_literals;

namespace
{

/// Serializes a message the ordinary typed way, so the converter's input is the same
/// CDR the middleware would hand a generic subscription.
template <typename MessageT>
rclcpp::SerializedMessage serialize(const MessageT& message)
{
  rclcpp::Serialization<MessageT> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&message, &serialized);
  return serialized;
}

dc_interfaces::msg::StringStamped make_string_stamped()
{
  dc_interfaces::msg::StringStamped message;
  message.header.stamp.sec = 1750000000;
  message.header.stamp.nanosec = 123456789;
  message.header.frame_id = "base_link";
  message.data = R"({"value": 42})";
  message.group_key = "synth00";
  return message;
}

RawConfig test_config(std::vector<std::string> include)
{
  RawConfig config;
  config.enabled = true;
  config.destination = "records";
  config.include = std::move(include);
  config.exclude = {};
  config.exclude_types = {};
  config.rescan_interval_secs = 0.2;
  config.max_rate_hz = 0.0;
  config.max_message_size_bytes = 0;
  return config;
}

/// Spins `node` until `predicate` holds or the deadline passes. Returns whether it held.
template <typename Predicate>
bool spin_until(const rclcpp::Node::SharedPtr& node, Predicate predicate, std::chrono::seconds timeout = 10s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (predicate())
    {
      return true;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}

}  // namespace

// --- introspection → JSON -----------------------------------------------------------

TEST(RawMessageConverterTest, ConvertsACustomNonStdMsgsType)
{
  RawMessageConverter converter("dc_interfaces/msg/StringStamped");
  const nlohmann::json payload = converter.to_json(serialize(make_string_stamped()));

  EXPECT_EQ(payload["data"], R"({"value": 42})");
  EXPECT_EQ(payload["group_key"], "synth00");
  EXPECT_EQ(payload["header"]["frame_id"], "base_link");
  EXPECT_EQ(payload["header"]["stamp"]["sec"], 1750000000);
  EXPECT_EQ(payload["header"]["stamp"]["nanosec"], 123456789);
}

TEST(RawMessageConverterTest, ConvertsNestedMessagesAndSequences)
{
  std_msgs::msg::Float64MultiArray message;
  message.data = { 1.5, -2.25, 0.0 };
  message.layout.data_offset = 7;
  std_msgs::msg::MultiArrayDimension dimension;
  dimension.label = "rows";
  dimension.size = 3;
  dimension.stride = 1;
  message.layout.dim.push_back(dimension);

  RawMessageConverter converter("std_msgs/msg/Float64MultiArray");
  const nlohmann::json payload = converter.to_json(serialize(message));

  EXPECT_EQ(payload["data"], (nlohmann::json{ 1.5, -2.25, 0.0 }));
  EXPECT_EQ(payload["layout"]["data_offset"], 7);
  ASSERT_EQ(payload["layout"]["dim"].size(), 1u);
  EXPECT_EQ(payload["layout"]["dim"][0]["label"], "rows");
  EXPECT_EQ(payload["layout"]["dim"][0]["size"], 3);
}

TEST(RawMessageConverterTest, RejectsAnUnknownType)
{
  EXPECT_THROW(RawMessageConverter("no_such_pkg/msg/NoSuchMessage"), RawConfigError);
}

TEST(RawStampTest, PrefersTheMessageHeaderStamp)
{
  RawMessageConverter converter("dc_interfaces/msg/StringStamped");
  const auto stamp = stamp_from_payload(converter.to_json(serialize(make_string_stamped())));
  ASSERT_TRUE(stamp.has_value());
  EXPECT_EQ(stamp->secs, 1750000000u);
  EXPECT_EQ(stamp->nanos, 123456789u);
}

TEST(RawStampTest, ReportsNoStampForAHeaderlessMessage)
{
  RawMessageConverter converter("std_msgs/msg/Float64MultiArray");
  EXPECT_FALSE(stamp_from_payload(converter.to_json(serialize(std_msgs::msg::Float64MultiArray()))).has_value());
  // Not a message at all, and messages whose `header` isn't a Header.
  EXPECT_FALSE(stamp_from_payload(nlohmann::json{ { "header", 3 } }).has_value());
  EXPECT_FALSE(stamp_from_payload(nlohmann::json{ { "header", { { "stamp", "now" } } } }).has_value());
}

// --- discovery + generic subscription on a live graph --------------------------------

TEST(RawSubscriptionManagerTest, DiscoversAndForwardsACustomTypeTopic)
{
  auto node = std::make_shared<rclcpp::Node>("raw_discovery_test");
  auto publisher = node->create_publisher<dc_interfaces::msg::StringStamped>("/raw_test/custom", 10);

  std::mutex mutex;
  std::vector<std::pair<std::string, nlohmann::json>> received;
  RawStats stats;
  RawSubscriptionManager manager(
      node.get(), test_config({ "^/raw_test/" }),
      [&](const std::string& tag, const RawStamp&, nlohmann::json payload) {
        std::lock_guard<std::mutex> lock(mutex);
        received.emplace_back(tag, std::move(payload));
        return true;
      },
      &stats);
  manager.start();

  ASSERT_TRUE(spin_until(node, [&] { return manager.subscription_count() == 1; }))
      << "the manager never discovered /raw_test/custom";

  ASSERT_TRUE(spin_until(node, [&] {
    publisher->publish(make_string_stamped());
    std::lock_guard<std::mutex> lock(mutex);
    return !received.empty();
  })) << "no raw Record reached the sink";

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(received.front().first, "dc.raw.raw_test.custom");
  EXPECT_EQ(received.front().second["group_key"], "synth00");
  EXPECT_GT(stats.forwarded.load(), 0u);
}

TEST(RawSubscriptionManagerTest, PicksUpATopicThatAppearsAfterStartup)
{
  auto node = std::make_shared<rclcpp::Node>("raw_rescan_test");
  RawStats stats;
  RawSubscriptionManager manager(
      node.get(), test_config({ "^/raw_test_late/" }),
      [](const std::string&, const RawStamp&, nlohmann::json) { return true; }, &stats);
  manager.start();
  EXPECT_EQ(manager.subscription_count(), 0u);

  // The publisher only exists now — the whole point of the periodic rescan.
  auto publisher = node->create_publisher<dc_interfaces::msg::StringStamped>("/raw_test_late/appeared", 10);
  EXPECT_TRUE(spin_until(node, [&] { return manager.subscription_count() == 1; }));
}

TEST(RawSubscriptionManagerTest, RespectsTheFilterOnDiscoveredTopics)
{
  auto node = std::make_shared<rclcpp::Node>("raw_filter_test");
  auto kept = node->create_publisher<dc_interfaces::msg::StringStamped>("/raw_test_filter/kept", 10);
  auto dropped = node->create_publisher<dc_interfaces::msg::StringStamped>("/raw_test_filter/dropped", 10);

  RawConfig config = test_config({ "^/raw_test_filter/" });
  config.exclude = { "dropped$" };
  RawStats stats;
  RawSubscriptionManager manager(
      node.get(), config, [](const std::string&, const RawStamp&, nlohmann::json) { return true; }, &stats);
  manager.start();

  ASSERT_TRUE(spin_until(node, [&] { return manager.subscription_count() == 1; }));
  // Give a second rescan a chance to (wrongly) pick the excluded topic up.
  spin_until(
      node, [&] { return manager.subscription_count() > 1; }, 2s);
  EXPECT_EQ(manager.subscription_count(), 1u);
}

// --- firehose / backpressure ---------------------------------------------------------

TEST(RawSubscriptionManagerTest, RateLimitDecimatesAFirehoseTopic)
{
  auto node = std::make_shared<rclcpp::Node>("raw_firehose_test");
  auto publisher = node->create_publisher<dc_interfaces::msg::StringStamped>("/raw_test_fire/hose", 500);

  RawConfig config = test_config({ "^/raw_test_fire/" });
  config.max_rate_hz = 2.0;  // at most one Record every 500 ms
  // Deep enough that the burst below is actually delivered to the callback rather than
  // being discarded by the middleware's own history depth first — the point of this test
  // is the Bridge's decimation, not the DDS queue's. (With the default depth of 10 the
  // subscription only ever sees the last 10 of the 200, which is itself worth knowing:
  // `raw.qos_depth` bounds how much a momentarily busy Bridge can buffer per topic.)
  config.qos_depth = 500;
  RawStats stats;
  std::atomic<std::size_t> forwarded{ 0 };
  RawSubscriptionManager manager(
      node.get(), config,
      [&](const std::string&, const RawStamp&, nlohmann::json) {
        ++forwarded;
        return true;
      },
      &stats);
  manager.start();
  ASSERT_TRUE(spin_until(node, [&] { return manager.subscription_count() == 1; }));

  // A burst far faster than the limit: everything must still arrive at the subscription
  // (so the drops are the Bridge's deliberate decision, not lost messages) and almost
  // all of it must be shed before it reaches the Shipper.
  for (int i = 0; i < 200; ++i)
  {
    publisher->publish(make_string_stamped());
  }
  spin_until(
      node, [&] { return stats.dropped_rate.load() >= 150; }, 5s);

  EXPECT_GT(stats.dropped_rate.load(), 100u) << "the firehose was not decimated";
  EXPECT_LE(forwarded.load(), 5u) << "far more Records were forwarded than the 2 Hz limit allows";
}

TEST(RawSubscriptionManagerTest, DropsRatherThanQueuesWhenTheShipperRefuses)
{
  auto node = std::make_shared<rclcpp::Node>("raw_backpressure_test");
  auto publisher = node->create_publisher<dc_interfaces::msg::StringStamped>("/raw_test_bp/hose", 200);

  RawStats stats;
  // The sink stands in for a Forwarder that is refusing everything (Vector unreachable,
  // or its socket blocked past write_timeout). The documented behaviour is that raw
  // Records are dropped and counted — never buffered inside the Bridge, which is what
  // would turn a firehose into unbounded memory growth.
  RawSubscriptionManager manager(
      node.get(), test_config({ "^/raw_test_bp/" }),
      [](const std::string&, const RawStamp&, nlohmann::json) { return false; }, &stats);
  manager.start();
  ASSERT_TRUE(spin_until(node, [&] { return manager.subscription_count() == 1; }));

  for (int i = 0; i < 50; ++i)
  {
    publisher->publish(make_string_stamped());
  }
  ASSERT_TRUE(spin_until(
      node, [&] { return stats.dropped_shipper.load() >= 10; }, 5s));
  EXPECT_EQ(stats.forwarded.load(), 0u);
}

TEST(RawSubscriptionManagerTest, DropsMessagesOverTheSizeLimitBeforeDeserializing)
{
  auto node = std::make_shared<rclcpp::Node>("raw_oversize_test");
  auto publisher = node->create_publisher<dc_interfaces::msg::StringStamped>("/raw_test_size/big", 10);

  RawConfig config = test_config({ "^/raw_test_size/" });
  config.max_message_size_bytes = 64;
  RawStats stats;
  RawSubscriptionManager manager(
      node.get(), config, [](const std::string&, const RawStamp&, nlohmann::json) { return true; }, &stats);
  manager.start();
  ASSERT_TRUE(spin_until(node, [&] { return manager.subscription_count() == 1; }));

  auto big = make_string_stamped();
  big.data = std::string(4096, 'x');
  ASSERT_TRUE(spin_until(node, [&] {
    publisher->publish(big);
    return stats.dropped_oversize.load() > 0;
  }));
  EXPECT_EQ(stats.forwarded.load(), 0u);
}

TEST(RawSubscriptionManagerTest, RejectsAnEnabledModeWithNoDestination)
{
  auto node = std::make_shared<rclcpp::Node>("raw_no_destination_test");
  RawConfig config = test_config({ "^/" });
  config.destination.clear();
  EXPECT_THROW(RawSubscriptionManager(
                   node.get(), config, [](const std::string&, const RawStamp&, nlohmann::json) { return true; },
                   nullptr),
               RawConfigError);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
