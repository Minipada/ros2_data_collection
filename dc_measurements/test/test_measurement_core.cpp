// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "dc_measurements/measurement_core.hpp"

using dc_measurements::LogLevel;
using dc_measurements::MeasurementCore;
using dc_measurements::RecordOut;

// Exercises the ROS-free Measurement pipeline (#499) the way test_incident_releaser.cpp exercises
// the state machine: no rclcpp, no node -- time is passed in, Records come back through a
// callback, Conditions are stand-in lookups.

namespace
{

// Seconds after the epoch, so throttle windows are plain arithmetic.
MeasurementCore::TimePoint at(const double seconds)
{
  return MeasurementCore::TimePoint{} +
         std::chrono::duration_cast<MeasurementCore::TimePoint::duration>(std::chrono::duration<double>(seconds));
}

// Everything the core reports, standing in for the ROS driver's publisher and logger.
struct Captured
{
  std::vector<RecordOut> records;
  std::vector<std::pair<LogLevel, std::string>> logs;
  int validation_failures{ 0 };

  MeasurementCore::PublishFn publishFn()
  {
    return [this](const RecordOut& out) { records.push_back(out); };
  }

  MeasurementCore::LogFn logFn()
  {
    return [this](const LogLevel level, const std::string& message) { logs.emplace_back(level, message); };
  }

  MeasurementCore::ValidationFailureFn failureFn()
  {
    return [this](const json&) { validation_failures++; };
  }

  int countLevel(const LogLevel level) const
  {
    int count = 0;
    for (const auto& [logged_level, message] : logs)
    {
      (void)message;
      if (logged_level == level)
      {
        count++;
      }
    }
    return count;
  }

  bool sawMessage(const std::string& needle) const
  {
    for (const auto& [level, message] : logs)
    {
      (void)level;
      if (message.find(needle) != std::string::npos)
      {
        return true;
      }
    }
    return false;
  }
};

// A counting stand-in for the driver's Condition resolution.
struct FakeConditions
{
  bool state{ false };
  int calls{ 0 };

  MeasurementCore::ConditionStateLookup lookup()
  {
    return [this](const std::string&) {
      calls++;
      return state;
    };
  }
};

MeasurementCore::Config baseConfig()
{
  MeasurementCore::Config config;
  config.measurement_name = "dummy";
  config.measurement_plugin = "dc_measurements/Dummy";
  config.group_key = "gk";
  config.run_id = "run-1";
  config.include_measurement_name = true;
  return config;
}

class MeasurementCoreTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_, ec);
  }

  std::filesystem::path tmp_ =
      std::filesystem::temp_directory_path() / ("dc_measurement_core_test_" + std::to_string(::getpid()));
};

TEST_F(MeasurementCoreTest, PublishesTheEnrichedRecord)
{
  Captured captured;
  MeasurementCore core;
  core.configure(baseConfig(), captured.publishFn(), captured.failureFn(), captured.logFn());

  FakeConditions conditions;
  core.publish(R"({"message":"hi"})", "gk", 42, [](const std::string&) { return true; }, conditions.lookup(), at(100.0));

  ASSERT_EQ(captured.records.size(), 1u);
  const RecordOut& out = captured.records[0];
  EXPECT_EQ(out.group_key, "gk");
  EXPECT_EQ(out.stamp_ns, 42);
  const json record = json::parse(out.data);
  EXPECT_EQ(record["message"], "hi");
  EXPECT_EQ(record["name"], "dummy");
  EXPECT_EQ(record["run_id"], "run-1");
  // No Condition is configured: the lookup is never consulted, exactly like the old
  // isAnyConditionSet() && isConditionOn() short-circuit.
  EXPECT_EQ(conditions.calls, 0);
}

TEST_F(MeasurementCoreTest, DropsEmptyPayloadsWithAThrottledWarning)
{
  Captured captured;
  MeasurementCore core;
  core.configure(baseConfig(), captured.publishFn(), captured.failureFn(), captured.logFn());

  const MeasurementCore::ConditionStateLookup no_gate = [](const std::string&) { return true; };
  FakeConditions conditions;

  core.publish("", "gk", 0, no_gate, conditions.lookup(), at(100.0));
  core.publish("null", "gk", 0, no_gate, conditions.lookup(), at(105.0));
  core.publish("", "gk", 0, no_gate, conditions.lookup(), at(109.0));
  core.publish("", "gk", 0, no_gate, conditions.lookup(), at(111.0));

  EXPECT_TRUE(captured.records.empty());
  EXPECT_EQ(captured.countLevel(LogLevel::Warn), 2);
  EXPECT_TRUE(captured.sawMessage("No data collected from measurement dummy"));
}

TEST_F(MeasurementCoreTest, ReportsUnparsablePayloadsAndPublishesThemAsTheyCame)
{
  Captured captured;
  MeasurementCore core;
  core.configure(baseConfig(), captured.publishFn(), captured.failureFn(), captured.logFn());

  core.publish(
      "not json", "gk", 7, [](const std::string&) { return true; }, [](const std::string&) { return false; }, at(100.0));

  ASSERT_EQ(captured.records.size(), 1u);
  EXPECT_EQ(captured.records[0].data, "not json");
  EXPECT_TRUE(captured.sawMessage("Error parsing JSON while enriching: not json"));
}

TEST_F(MeasurementCoreTest, GateLatchesOpenAndStopsConsultingTheCondition)
{
  Captured captured;
  MeasurementCore core;
  auto config = baseConfig();
  config.gate_condition = "go";
  core.configure(config, captured.publishFn(), captured.failureFn(), captured.logFn());

  FakeConditions gate;  // the gate condition reads false to start with
  const auto publish_once = [&](const MeasurementCore::TimePoint& now) {
    core.publish(R"({"message":"x"})", "gk", 0, gate.lookup(), [](const std::string&) { return false; }, now);
  };

  publish_once(at(100.0));
  EXPECT_TRUE(captured.records.empty());
  EXPECT_EQ(gate.calls, 1);

  // The gate latches on the first true reading...
  gate.state = true;
  publish_once(at(101.0));
  ASSERT_EQ(captured.records.size(), 1u);
  EXPECT_TRUE(captured.sawMessage("gate_condition 'go' became true"));

  // ...and from then on the condition is never consulted again, even reading false.
  gate.state = false;
  const int calls_before = gate.calls;
  publish_once(at(102.0));
  ASSERT_EQ(captured.records.size(), 2u);
  EXPECT_EQ(gate.calls, calls_before);
}

TEST_F(MeasurementCoreTest, UnknownGateConditionHoldsAllCollectionBack)
{
  Captured captured;
  MeasurementCore core;
  auto config = baseConfig();
  config.gate_condition = "go";
  core.configure(config, captured.publishFn(), captured.failureFn(), captured.logFn());

  // What the driver's lookup does for a gate_condition naming no configured Condition: report
  // false, so the gate never opens.
  const MeasurementCore::ConditionStateLookup unknown = [](const std::string&) { return false; };

  core.publish(R"({"message":"x"})", "gk", 0, unknown, [](const std::string&) { return false; }, at(100.0));
  core.publish(R"({"message":"x"})", "gk", 0, unknown, [](const std::string&) { return false; }, at(101.0));

  EXPECT_TRUE(captured.records.empty());
}

TEST_F(MeasurementCoreTest, InitQuotaPublishesUnconditionallyThenConditionsCap)
{
  Captured captured;
  auto config = baseConfig();
  config.init_max_measurements = 1;
  config.condition_max_measurements = 2;
  config.if_any_conditions = { "moving" };
  MeasurementCore core;
  core.configure(config, captured.publishFn(), captured.failureFn(), captured.logFn());

  FakeConditions conditions;
  conditions.state = true;
  const auto publish_once = [&] {
    core.publish(R"({"message":"x"})", "gk", 0, [](const std::string&) { return true; }, conditions.lookup(), at(100.0));
  };

  publish_once();  // the init quota, unconditional
  EXPECT_EQ(captured.records.size(), 1u);
  EXPECT_GT(conditions.calls, 0);

  publish_once();
  publish_once();  // within the condition cap
  publish_once();  // over it
  EXPECT_EQ(captured.records.size(), 3u);

  // A false reading resets the cap for the next true stretch.
  conditions.state = false;
  publish_once();
  EXPECT_EQ(captured.records.size(), 3u);
  conditions.state = true;
  publish_once();
  publish_once();
  publish_once();
  EXPECT_EQ(captured.records.size(), 5u);
}

TEST_F(MeasurementCoreTest, CollectionFinishedAfterTheQuotaWithNoConditionPublishing)
{
  Captured captured;
  auto config = baseConfig();
  config.init_max_measurements = 2;
  config.condition_max_measurements = -1;
  MeasurementCore core;
  core.configure(config, captured.publishFn(), captured.failureFn(), captured.logFn());

  const MeasurementCore::ConditionStateLookup no_gate = [](const std::string&) { return true; };
  EXPECT_FALSE(core.collectionFinished());
  core.publish(R"({"message":"x"})", "gk", 0, no_gate, [](const std::string&) { return false; }, at(100.0));
  EXPECT_FALSE(core.collectionFinished());
  core.publish(R"({"message":"x"})", "gk", 0, no_gate, [](const std::string&) { return false; }, at(101.0));
  EXPECT_TRUE(core.collectionFinished());
  EXPECT_EQ(captured.records.size(), 2u);
}

TEST_F(MeasurementCoreTest, EnabledStateMachineOwnsCollectionExplicitly)
{
  Captured captured;
  MeasurementCore core;
  core.configure(baseConfig(), captured.publishFn(), captured.failureFn(), captured.logFn());

  EXPECT_TRUE(core.collectible());
  core.setEnabled(false);
  EXPECT_FALSE(core.collectible());
  core.setEnabled(true);
  EXPECT_TRUE(core.collectible());

  // A failed onConfigure() hook is permanent: however many activate() cycles follow, the
  // Measurement stays silent.
  core.markConfigureFailed();
  EXPECT_FALSE(core.collectible());
  core.setEnabled(true);
  EXPECT_FALSE(core.collectible());
}

TEST_F(MeasurementCoreTest, IncidentBufferReleasesOnFlush)
{
  Captured captured;
  auto config = baseConfig();
  config.buffer_duration_sec = 60.0;
  config.cooldown_sec = 50.0;
  MeasurementCore core;
  core.configure(config, captured.publishFn(), captured.failureFn(), captured.logFn());

  EXPECT_TRUE(core.hasReleaser());
  core.offerSample(R"({"message":"a"})", at(100.0));
  core.offerSample(R"({"message":"b"})", at(101.0));
  EXPECT_TRUE(captured.records.empty());

  core.onFlushEvent("inc-1", at(102.0));
  ASSERT_EQ(captured.records.size(), 2u);
  for (const RecordOut& out : captured.records)
  {
    const json record = json::parse(out.data);
    // The Incident rides the typed envelope field (#506); the payload carries no such key.
    EXPECT_EQ(out.incident_id, "inc-1");
    EXPECT_FALSE(record.contains("incident_id"));
    EXPECT_EQ(out.group_key, "gk");
    // Released Records are stamped with when they were collected, not when they were released.
    EXPECT_NE(out.stamp_ns, std::chrono::duration_cast<std::chrono::nanoseconds>(at(102.0).time_since_epoch()).count());
  }
  EXPECT_EQ(json::parse(captured.records[0].data)["message"], "a");
  EXPECT_EQ(json::parse(captured.records[1].data)["message"], "b");

  // Cooldown: a flapping Trigger's second event starts no new cycle.
  core.offerSample(R"({"message":"c"})", at(103.0));
  core.onFlushEvent("inc-2", at(104.0));
  EXPECT_EQ(captured.records.size(), 2u);
}

TEST_F(MeasurementCoreTest, PostRollPublishesLiveThenReArms)
{
  Captured captured;
  auto config = baseConfig();
  config.buffer_duration_sec = 60.0;
  config.post_roll_duration_sec = 30.0;
  MeasurementCore core;
  core.configure(config, captured.publishFn(), captured.failureFn(), captured.logFn());

  core.onFlushEvent("inc-1", at(100.0));  // empty buffer, straight into post-roll
  core.offerSample(R"({"message":"live"})", at(110.0));
  ASSERT_EQ(captured.records.size(), 1u);
  const json record = json::parse(captured.records[0].data);
  EXPECT_EQ(record["message"], "live");
  EXPECT_EQ(captured.records[0].incident_id, "inc-1");
  EXPECT_FALSE(record.contains("incident_id"));

  // Past the post-roll deadline the next sample buffers again.
  core.offerSample(R"({"message":"buffered"})", at(131.0));
  EXPECT_EQ(captured.records.size(), 1u);
}

TEST_F(MeasurementCoreTest, StagesRecordFilesIntoTheScratchRing)
{
  Captured captured;
  auto config = baseConfig();
  config.measurement_name = "camera";
  config.buffer_duration_sec = 60.0;
  // A dated save tree: the scratch ring sits on its literal prefix, outside the rolling part.
  config.save_local_base_path_expanded = (tmp_ / "%Y/%M/%D/%H").string();
  MeasurementCore core;
  core.configure(config, captured.publishFn(), captured.failureFn(), captured.logFn());

  const auto produced = tmp_ / "%Y/%M/%D/%H" / "frame.jpg";
  std::filesystem::create_directories(produced.parent_path());
  {
    std::ofstream f(produced);
    f << "jpeg-bytes";
  }

  core.offerSample(R"({"local_paths":{"jpeg":")" + produced.string() +
                       R"("},"remote_paths":{"minio":{"jpeg":"cam/frame.jpg"}}})",
                   at(100.0));

  // Staging *moves* the File...
  EXPECT_FALSE(std::filesystem::exists(produced));
  const auto scratch_root = tmp_ / ".dc_incident_scratch" / "camera";
  ASSERT_TRUE(std::filesystem::exists(scratch_root));
  const auto staged = *(std::filesystem::directory_iterator(scratch_root));
  EXPECT_EQ(staged.path().filename().string().find("frame.jpg") != std::string::npos, true);

  // ...and the buffered Record is rewritten to point at the staged copy, remote key untouched.
  core.onFlushEvent("inc-1", at(101.0));
  ASSERT_EQ(captured.records.size(), 1u);
  const json record = json::parse(captured.records[0].data);
  EXPECT_EQ(record["local_paths"]["jpeg"].get<std::string>(), staged.path().string());
  EXPECT_EQ(record["remote_paths"]["minio"]["jpeg"], "cam/frame.jpg");
}

TEST_F(MeasurementCoreTest, TeardownPurgesStagedFilesOfBufferedRecords)
{
  Captured captured;
  auto config = baseConfig();
  config.buffer_duration_sec = 60.0;
  config.save_local_base_path_expanded = tmp_.string();
  MeasurementCore core;
  core.configure(config, captured.publishFn(), captured.failureFn(), captured.logFn());

  const auto produced = tmp_ / "frame.jpg";
  {
    std::ofstream f(produced);
    f << "jpeg-bytes";
  }
  core.offerSample(R"({"local_paths":{"jpeg":")" + produced.string() + R"("}})", at(100.0));

  const auto scratch_root = tmp_ / ".dc_incident_scratch" / "dummy";
  ASSERT_TRUE(std::filesystem::exists(scratch_root));
  core.teardown();
  // The buffered Record is gone with the releaser, so its staged File would be orphaned there.
  EXPECT_TRUE(std::filesystem::is_empty(scratch_root));
}

TEST_F(MeasurementCoreTest, SchemaValidationFeedsThePluginHookAndStillPublishes)
{
  Captured captured;
  auto config = baseConfig();
  config.enable_validator = true;
  MeasurementCore core;
  core.configure(config, captured.publishFn(), captured.failureFn(), captured.logFn());

  const auto schema = tmp_ / "dummy.json";
  std::filesystem::create_directories(tmp_);
  {
    std::ofstream f(schema);
    f << R"({"type":"object","required":["name"]})";
  }
  core.validateSchema(schema.string());
  EXPECT_FALSE(core.schemaEmpty());

  // A Record the schema rejects is reported through the hook, but still published, exactly as
  // this chain always did.
  core.publish(R"({"other":1})", "gk", 0, [](const std::string&) { return true; },
               [](const std::string&) { return false; }, at(100.0));
  EXPECT_EQ(captured.validation_failures, 1);
  EXPECT_TRUE(captured.sawMessage("Validation failed:"));
  EXPECT_EQ(captured.records.size(), 1u);

  core.publish(R"({"name":"dummy"})", "gk", 0, [](const std::string&) { return true; },
               [](const std::string&) { return false; }, at(101.0));
  EXPECT_EQ(captured.validation_failures, 1);
  EXPECT_EQ(captured.records.size(), 2u);
}

TEST_F(MeasurementCoreTest, ValidateSchemaResolvesSiblingRefsAndReportsBadOnes)
{
  Captured captured;
  MeasurementCore core;
  core.configure(baseConfig(), captured.publishFn(), captured.failureFn(), captured.logFn());

  std::filesystem::create_directories(tmp_);
  {
    std::ofstream f(tmp_ / "common.json");
    f << R"({"type":"object"})";
  }
  const auto root = tmp_ / "root.json";
  {
    std::ofstream f(root);
    f << R"({"$ref":"common.json"})";
  }
  core.validateSchema(root.string());
  EXPECT_FALSE(core.schemaEmpty());

  const auto broken = tmp_ / "broken.json";
  {
    std::ofstream f(broken);
    f << R"({"$ref":"missing.json"})";
  }
  EXPECT_THROW(core.validateSchema(broken.string()), std::runtime_error);

  const auto unparsable = tmp_ / "unparsable.json";
  {
    std::ofstream f(unparsable);
    f << "not json";
  }
  core.validateSchema(unparsable.string());
  EXPECT_TRUE(captured.sawMessage("Error parsing JSON file json_schema_path"));
}

TEST(MeasurementCoreNamingTest, SnakeCaseKeepsAcronymsWithTheWordTheyPrecede)
{
  EXPECT_EQ(MeasurementCore::snakeCase("OS"), "os");
  EXPECT_EQ(MeasurementCore::snakeCase("TCPHealth"), "tcp_health");
  EXPECT_EQ(MeasurementCore::snakeCase("Ros2ControlStatus"), "ros2_control_status");
  EXPECT_EQ(MeasurementCore::snakeCase("MissionNav2ThroughPoses"), "mission_nav2_through_poses");
}

TEST(MeasurementCoreNamingTest, DefaultSchemaFileTakesTheTypeNotTheWholeLookupName)
{
  EXPECT_EQ(MeasurementCore::defaultSchemaFile("dc_measurements/Battery"), "battery.json");
  EXPECT_EQ(MeasurementCore::defaultSchemaFile("dc_measurements/TCPHealth"), "tcp_health.json");
  EXPECT_EQ(MeasurementCore::defaultSchemaFile("dc_demos/UptimeCustom"), "uptime_custom.json");
}

}  // namespace
