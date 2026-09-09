// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "measurement_test_bench.hpp"

class MeasurementDiagnosticsTest : public MeasurementBench
{
protected:
  MeasurementDiagnosticsTest() : MeasurementBench("diagnostics")
  {
    diag_pub_ =
        ms_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::SystemDefaultsQoS());
  }

  static diagnostic_msgs::msg::DiagnosticStatus makeStatus(const std::string& name, uint8_t level,
                                                           const std::string& message)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.level = level;
    status.message = message;
    status.hardware_id = "test_hw";

    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = "temperature";
    key_value.value = "42";
    status.values.push_back(key_value);

    return status;
  }

  void publishDiagnostics(const std::vector<diagnostic_msgs::msg::DiagnosticStatus>& statuses)
  {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = ms_node_->get_clock()->now();
    msg.status = statuses;
    diag_pub_->publish(msg);
  }

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
};

TEST_F(MeasurementDiagnosticsTest, FiltersByLevelThresholdAndPreservesValues)
{
  ms_node_->declare_parameter("diagnostics.plugin", std::string("dc_measurements/Diagnostics"));
  ms_node_->declare_parameter("diagnostics.group_key", std::string("diagnostics"));
  ms_node_->declare_parameter("diagnostics.topic_output", std::string("/dc/measurement/diagnostics"));
  ms_node_->declare_parameter("diagnostics.level_threshold", std::string("WARN"));
  ms_node_->declare_parameter("diagnostics.init_collect", false);

  startLifecycleNode();
  waitForSubscriber("/diagnostics");

  std::vector<diagnostic_msgs::msg::DiagnosticStatus> statuses;
  statuses.push_back(makeStatus("battery", diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery nominal"));
  statuses.push_back(makeStatus("motor_driver", diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor fault"));

  ASSERT_TRUE(spinUntil([&] {
    publishDiagnostics(statuses);
    return callback_active_;
  })) << "no Record ever arrived";

  ASSERT_EQ(data_json_["statuses"].size(), 1u);
  EXPECT_EQ(data_json_["statuses"][0]["name"], "motor_driver");
  EXPECT_EQ(data_json_["statuses"][0]["level"], diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(data_json_["statuses"][0]["values"]["temperature"], "42");
}

TEST_F(MeasurementDiagnosticsTest, NameAllowlistFiltersOutOtherNames)
{
  ms_node_->declare_parameter("diagnostics.plugin", std::string("dc_measurements/Diagnostics"));
  ms_node_->declare_parameter("diagnostics.group_key", std::string("diagnostics"));
  ms_node_->declare_parameter("diagnostics.topic_output", std::string("/dc/measurement/diagnostics"));
  ms_node_->declare_parameter("diagnostics.names", std::vector<std::string>{ "battery" });
  ms_node_->declare_parameter("diagnostics.init_collect", false);

  startLifecycleNode();
  waitForSubscriber("/diagnostics");

  std::vector<diagnostic_msgs::msg::DiagnosticStatus> statuses;
  statuses.push_back(makeStatus("battery", diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery nominal"));
  statuses.push_back(makeStatus("motor_driver", diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor fault"));

  ASSERT_TRUE(spinUntil([&] {
    publishDiagnostics(statuses);
    return callback_active_;
  })) << "no Record ever arrived";

  ASSERT_EQ(data_json_["statuses"].size(), 1u);
  EXPECT_EQ(data_json_["statuses"][0]["name"], "battery");
}

DC_MEASUREMENT_TEST_MAIN()
