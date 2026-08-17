// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Raw / generic-subscription mode policy (#227): the filter, the Tag derivation and the
// per-topic rate limit that is raw mode's first line of defence against a firehose
// topic. All ROS-free — these are the decisions the manager makes before it ever touches
// a message.
#include "dc_bridge/raw_config.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace dc_bridge;

namespace
{

RawConfig default_config()
{
  RawConfig config;
  config.enabled = true;
  config.destination = "records";
  config.include = { "^/" };
  config.exclude = default_raw_exclude();
  config.exclude_types = default_raw_exclude_types();
  return config;
}

}  // namespace

TEST(RawFilter, AcceptsEveryTopicByDefault)
{
  RawTopicFilter filter(default_config());
  EXPECT_TRUE(filter.accepts("/robot/state", "my_robot_msgs/msg/State"));
  EXPECT_TRUE(filter.accepts("/deeply/nested/topic", "std_msgs/msg/String"));
}

TEST(RawFilter, IncludePatternsAreTheAllowlist)
{
  RawConfig config = default_config();
  config.include = { "^/robot/", "^/diagnostics$" };
  RawTopicFilter filter(config);

  EXPECT_TRUE(filter.accepts("/robot/state", "std_msgs/msg/String"));
  EXPECT_TRUE(filter.accepts("/diagnostics", "std_msgs/msg/String"));
  EXPECT_EQ(filter.evaluate("/other/topic", "std_msgs/msg/String"), RawSkip::NotIncluded);
}

TEST(RawFilter, ExcludeVetoesAnInclude)
{
  RawConfig config = default_config();
  config.include = { "^/robot/" };
  config.exclude = { "/secret" };
  RawTopicFilter filter(config);

  EXPECT_TRUE(filter.accepts("/robot/state", "std_msgs/msg/String"));
  EXPECT_EQ(filter.evaluate("/robot/secret/key", "std_msgs/msg/String"), RawSkip::Excluded);
}

TEST(RawFilter, DefaultsKeepRosInfrastructureAndDcRecordTopicsOut)
{
  RawTopicFilter filter(default_config());

  // /rosout would feed the Bridge's own warnings back into the pipeline it warns about.
  EXPECT_EQ(filter.evaluate("/rosout", "rcl_interfaces/msg/Log"), RawSkip::Excluded);
  EXPECT_EQ(filter.evaluate("/parameter_events", "rcl_interfaces/msg/ParameterEvent"), RawSkip::Excluded);
  // Already shipped as Measurement/Group Records — raw would double every one of them.
  EXPECT_EQ(filter.evaluate("/dc/measurement/uptime", "dc_interfaces/msg/StringStamped"), RawSkip::Excluded);
  EXPECT_EQ(filter.evaluate("/dc/group/robot", "dc_interfaces/msg/StringStamped"), RawSkip::Excluded);
  // A non-Record topic under /dc is fair game.
  EXPECT_TRUE(filter.accepts("/dc/e2e/synth/synth00", "dc_interfaces/msg/StringStamped"));
}

TEST(RawFilter, DefaultsKeepHighRateSensorTypesOut)
{
  RawTopicFilter filter(default_config());

  // #227's third open question, answered by the defaults: yes, excluded — and by *type*,
  // because a camera topic is not reliably named anything in particular.
  EXPECT_EQ(filter.evaluate("/front/rgb", "sensor_msgs/msg/Image"), RawSkip::ExcludedType);
  EXPECT_EQ(filter.evaluate("/anything", "sensor_msgs/msg/CompressedImage"), RawSkip::ExcludedType);
  EXPECT_EQ(filter.evaluate("/lidar/points", "sensor_msgs/msg/PointCloud2"), RawSkip::ExcludedType);
  EXPECT_EQ(filter.evaluate("/tf", "tf2_msgs/msg/TFMessage"), RawSkip::ExcludedType);
  // Small, low-rate sensor messages stay in.
  EXPECT_TRUE(filter.accepts("/imu", "sensor_msgs/msg/Imu"));
  EXPECT_TRUE(filter.accepts("/battery", "sensor_msgs/msg/BatteryState"));
}

TEST(RawFilter, EmptyIncludeListMatchesNothing)
{
  RawConfig config = default_config();
  config.include.clear();
  RawTopicFilter filter(config);
  EXPECT_EQ(filter.evaluate("/robot/state", "std_msgs/msg/String"), RawSkip::NotIncluded);
}

TEST(RawFilter, RejectsAnUncompilableRegexAtConstruction)
{
  RawConfig config = default_config();
  config.include = { "^/robot/[" };
  EXPECT_THROW(RawTopicFilter{ config }, RawConfigError);
}

TEST(RawTag, DerivesFromTheTopicUnderTheRawNamespace)
{
  EXPECT_EQ(raw_tag(RAW_TAG_PREFIX, "/dc/e2e/synth/synth00"), "dc.raw.dc.e2e.synth.synth00");
  EXPECT_EQ(raw_tag(RAW_TAG_PREFIX, "/imu"), "dc.raw.imu");
  // No leading slash (never produced by the ROS graph, but the derivation shouldn't care).
  EXPECT_EQ(raw_tag(RAW_TAG_PREFIX, "imu"), "dc.raw.imu");
  EXPECT_EQ(raw_tag("raw.", "/robot/state"), "raw.robot.state");
}

TEST(RawTag, SanitisesAnythingOutsideTheTagAlphabet)
{
  // A Tag ends up in a VRL string literal and a Vector route name; nothing outside
  // [A-Za-z0-9_.] survives the derivation.
  EXPECT_EQ(raw_tag(RAW_TAG_PREFIX, "/robot state"), "dc.raw.robot_state");
  EXPECT_EQ(raw_tag(RAW_TAG_PREFIX, "/robot\"state"), "dc.raw.robot_state");
  EXPECT_EQ(raw_tag(RAW_TAG_PREFIX, "/a-b"), "dc.raw.a_b");
  EXPECT_EQ(raw_tag(RAW_TAG_PREFIX, "/ok_1/ok_2"), "dc.raw.ok_1.ok_2");
}

TEST(RawRateLimiter, DecimatesAFirehoseToTheConfiguredRate)
{
  // The firehose case (#227's second open question): 1000 messages arriving over one
  // simulated second on a 10 Hz limit must not produce 1000 Records.
  RawRateLimiter limiter(10.0);
  const auto start = RawRateLimiter::Clock::now();
  std::size_t passed = 0;
  for (int i = 0; i < 1000; ++i)
  {
    if (limiter.allow("/firehose", start + std::chrono::milliseconds(i)))
    {
      ++passed;
    }
  }
  // One per 100 ms across a 1 s window, plus the first message.
  EXPECT_GE(passed, 9u);
  EXPECT_LE(passed, 11u);
}

TEST(RawRateLimiter, LimitsAreIndependentPerTopic)
{
  RawRateLimiter limiter(10.0);
  const auto now = RawRateLimiter::Clock::now();
  EXPECT_TRUE(limiter.allow("/a", now));
  EXPECT_TRUE(limiter.allow("/b", now));
  EXPECT_FALSE(limiter.allow("/a", now + std::chrono::milliseconds(1)));
  EXPECT_FALSE(limiter.allow("/b", now + std::chrono::milliseconds(1)));
  EXPECT_TRUE(limiter.allow("/a", now + std::chrono::milliseconds(200)));
}

TEST(RawRateLimiter, ZeroOrNegativeRateMeansUnlimited)
{
  RawRateLimiter limiter(0.0);
  EXPECT_TRUE(limiter.unlimited());
  const auto now = RawRateLimiter::Clock::now();
  for (int i = 0; i < 100; ++i)
  {
    EXPECT_TRUE(limiter.allow("/firehose", now));
  }
  EXPECT_TRUE(RawRateLimiter(-1.0).unlimited());
}

TEST(RawStatsTest, SummaryReportsEveryDropReason)
{
  RawStats stats;
  stats.subscribed_topics.store(3);
  stats.forwarded.store(10);
  stats.dropped_rate.store(4);
  stats.dropped_oversize.store(2);
  stats.dropped_shipper.store(1);
  stats.dropped_undecodable.store(5);

  const std::string summary = stats.summary();
  EXPECT_NE(summary.find("3 topic(s)"), std::string::npos);
  EXPECT_NE(summary.find("10 forwarded"), std::string::npos);
  EXPECT_NE(summary.find("4 rate"), std::string::npos);
  EXPECT_NE(summary.find("2 oversize"), std::string::npos);
  EXPECT_NE(summary.find("1 shipper"), std::string::npos);
  EXPECT_NE(summary.find("5 undecodable"), std::string::npos);
}
