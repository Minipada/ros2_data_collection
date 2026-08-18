// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <nlohmann/json-schema.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

class MeasurementFaultTest : public ::testing::Test
{
protected:
  MeasurementFaultTest()
  {
    SetUp();
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "fault" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/fault", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementFaultTest::dataCallback, this, std::placeholders::_1));
    diag_pub_ = ms_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/test/diagnostics",
                                                                                  rclcpp::SystemDefaultsQoS());
  }

  void TearDown() override
  {
    stopCollection();
  }

  // Idempotent: one test stops collection mid-fault on purpose, and TearDown must not then drive
  // the lifecycle node through a transition it is no longer in a state for.
  void stopCollection()
  {
    if (stopped_)
    {
      return;
    }
    stopped_ = true;
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("fault.plugin", std::string("dc_measurements/Fault"));
    ms_node_->declare_parameter("fault.group_key", std::string("fault"));
    ms_node_->declare_parameter("fault.topic_output", std::string("/dc/measurement/fault"));
    ms_node_->declare_parameter("fault.topic", std::string("/test/diagnostics"));
    ms_node_->declare_parameter("fault.polling_interval", 50);
    ms_node_->declare_parameter("fault.init_collect", false);
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  void dataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    records_.push_back(nlohmann::json::parse(data_str));
    callback_active_ = true;
  }

  static diagnostic_msgs::msg::DiagnosticStatus makeStatus(const std::string& name, uint8_t level,
                                                           const std::string& message)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.level = level;
    status.message = message;
    return status;
  }

  void publishDiagnostics(const diagnostic_msgs::msg::DiagnosticStatus& status)
  {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = ms_node_->get_clock()->now();
    msg.status.push_back(status);
    diag_pub_->publish(msg);
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void waitForSubscriber(const std::string& topic)
  {
    while (ms_node_->count_subscribers(topic) == 0)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
    }
  }

  // The detector treats a component's very first observed sample as a baseline, not a transition
  // (matching every other StateTransitionDetector consumer): no Record comes out of it, and the
  // level it carries never fires again since nothing is left to compare against. Every test that
  // wants to observe an actual "raise" therefore has to establish OK as the baseline first.
  void establishOkBaseline(const std::string& name)
  {
    publishDiagnostics(makeStatus(name, diagnostic_msgs::msg::DiagnosticStatus::OK, "nominal"));
    spinFor(std::chrono::milliseconds(100));
  }

  // Republishes `status` until a *new* Record matching `predicate` shows up, so a best-effort
  // sample lost before the plugin subscribed doesn't make the test flaky.
  nlohmann::json publishUntilRecord(const diagnostic_msgs::msg::DiagnosticStatus& status,
                                    const std::function<bool(const nlohmann::json&)>& predicate)
  {
    const size_t first_new = records_.size();
    for (int i = 0; i < 400; ++i)
    {
      publishDiagnostics(status);
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      for (size_t r = first_new; r < records_.size(); ++r)
      {
        if (predicate(records_[r]))
        {
          return records_[r];
        }
      }
    }
    ADD_FAILURE() << "No matching Record within the timeout";
    return nlohmann::json{};
  }

  static std::function<bool(const nlohmann::json&)> isEvent(const std::string& event)
  {
    return [event](const nlohmann::json& record) { return record.value("event", "") == event; };
  }

  // The schema the plugin itself validates against, applied here directly so a Record that only
  // half fills it fails the test rather than only logging.
  static void expectValidatesAgainstSchema(const nlohmann::json& record)
  {
    const std::string path =
        ament_index_cpp::get_package_share_directory("dc_measurements") + "/plugins/measurements/json/fault.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::vector<nlohmann::json> records_;

  bool stopped_{ false };

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementFaultTest, LevelChangeRaisesAnOpenFaultRecord)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/diagnostics");

  // Staying OK must not produce a Record.
  const auto ok = makeStatus("motor_driver", diagnostic_msgs::msg::DiagnosticStatus::OK, "Motor nominal");
  publishDiagnostics(ok);
  spinFor(std::chrono::milliseconds(200));
  ASSERT_TRUE(records_.empty()) << "A component that stays put must not produce a Record";

  const auto error = makeStatus("motor_driver", diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor fault");
  const auto raise = publishUntilRecord(error, isEvent("raise"));

  EXPECT_EQ(raise["seq"], 1);
  EXPECT_EQ(raise["component"], "motor_driver");
  EXPECT_EQ(raise["from_level"], "OK");
  EXPECT_EQ(raise["to_level"], "ERROR");
  EXPECT_EQ(raise["reason"], "Motor fault");
  EXPECT_EQ(raise["state"], "open");
  EXPECT_GT(raise["previous_level_duration_s"].get<double>(), 0.0);
  ASSERT_TRUE(raise.contains("fault_started_at"));
  EXPECT_FALSE(raise["fault_started_at"].get<std::string>().empty());
  EXPECT_FALSE(raise.contains("duration_s")) << "An open fault has no duration yet";
  expectValidatesAgainstSchema(raise);
}

TEST_F(MeasurementFaultTest, ReturningToOkClearsTheFaultAndReportsItsDuration)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/diagnostics");
  establishOkBaseline("motor_driver");

  const auto error = makeStatus("motor_driver", diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor fault");
  const auto raise = publishUntilRecord(error, isEvent("raise"));

  const auto ok = makeStatus("motor_driver", diagnostic_msgs::msg::DiagnosticStatus::OK, "Motor nominal");
  const auto clear = publishUntilRecord(ok, isEvent("clear"));

  EXPECT_EQ(clear["seq"], 2);
  EXPECT_EQ(clear["from_level"], "ERROR");
  EXPECT_EQ(clear["to_level"], "OK");
  EXPECT_EQ(clear["state"], "closed");
  ASSERT_TRUE(clear.contains("duration_s"));
  EXPECT_GT(clear["duration_s"].get<double>(), 0.0);
  // Both Records name the same fault, so the pair can be joined downstream.
  EXPECT_EQ(clear["fault_started_at"], raise["fault_started_at"]);
  expectValidatesAgainstSchema(clear);
}

TEST_F(MeasurementFaultTest, WorseningLevelIsAChangeNotAClearAndKeepsTheFaultOpen)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/diagnostics");
  establishOkBaseline("motor_driver");

  const auto warn = makeStatus("motor_driver", diagnostic_msgs::msg::DiagnosticStatus::WARN, "Motor warm");
  const auto raise = publishUntilRecord(warn, isEvent("raise"));
  ASSERT_EQ(raise["to_level"], "WARN");

  const auto error = makeStatus("motor_driver", diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor overheating");
  const auto change = publishUntilRecord(error, isEvent("change"));

  EXPECT_EQ(change["from_level"], "WARN");
  EXPECT_EQ(change["to_level"], "ERROR");
  EXPECT_EQ(change["state"], "open");
  EXPECT_FALSE(change.contains("duration_s")) << "The fault is still open, not cleared";
  // Still the same fault as the original raise: its start does not move.
  EXPECT_EQ(change["fault_started_at"], raise["fault_started_at"]);
  expectValidatesAgainstSchema(change);
}

TEST_F(MeasurementFaultTest, FlappingComponentGetsOneRecordPerTransitionWithIncreasingSequence)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/diagnostics");
  establishOkBaseline("wifi");

  const auto ok = makeStatus("wifi", diagnostic_msgs::msg::DiagnosticStatus::OK, "Link up");
  const auto warn = makeStatus("wifi", diagnostic_msgs::msg::DiagnosticStatus::WARN, "Weak signal");

  publishUntilRecord(warn, isEvent("raise"));
  publishUntilRecord(ok, isEvent("clear"));
  publishUntilRecord(warn, isEvent("raise"));
  publishUntilRecord(ok, isEvent("clear"));

  ASSERT_EQ(records_.size(), 4u);
  std::vector<std::string> events;
  for (const auto& record : records_)
  {
    events.push_back(record["event"].get<std::string>());
  }
  EXPECT_EQ(events, (std::vector<std::string>{ "raise", "clear", "raise", "clear" }));
  for (size_t i = 0; i < records_.size(); ++i)
  {
    EXPECT_EQ(records_[i]["seq"].get<uint64_t>(), i + 1);
  }
}

TEST_F(MeasurementFaultTest, FaultOpenAtShutdownStaysOpenAndReportsNoDuration)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForSubscriber("/test/diagnostics");
  establishOkBaseline("motor_driver");

  const auto error = makeStatus("motor_driver", diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor fault");
  publishUntilRecord(error, isEvent("raise"));

  // Collection stops with the fault still open.
  stopCollection();
  spinFor(std::chrono::milliseconds(200));

  ASSERT_EQ(records_.size(), 1u) << "Shutdown must not invent a clearing Record";
  EXPECT_EQ(records_.back()["state"], "open");
  EXPECT_FALSE(records_.back().contains("duration_s"))
      << "An unterminated fault must not report a duration that could be read as a zero-length outage";
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
