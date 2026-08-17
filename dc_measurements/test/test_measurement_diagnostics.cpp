// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

class MeasurementDiagnosticsTest : public ::testing::Test
{
protected:
  MeasurementDiagnosticsTest()
  {
    SetUp();
  }

  ~MeasurementDiagnosticsTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "diagnostics" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/diagnostics", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementDiagnosticsTest::diagnosticsDataCallback, this, std::placeholders::_1));
    diag_pub_ =
        ms_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::SystemDefaultsQoS());
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  void diagnosticsDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
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

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  nlohmann::json data_json_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementDiagnosticsTest, FiltersByLevelThresholdAndPreservesValues)
{
  ms_node_->declare_parameter("diagnostics.plugin", std::string("dc_measurements/Diagnostics"));
  ms_node_->declare_parameter("diagnostics.group_key", std::string("diagnostics"));
  ms_node_->declare_parameter("diagnostics.topic_output", std::string("/dc/measurement/diagnostics"));
  ms_node_->declare_parameter("diagnostics.level_threshold", std::string("WARN"));
  ms_node_->declare_parameter("diagnostics.init_collect", false);

  startLifecycleNode();

  while (ms_node_->count_subscribers("/diagnostics") == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  std::vector<diagnostic_msgs::msg::DiagnosticStatus> statuses;
  statuses.push_back(makeStatus("battery", diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery nominal"));
  statuses.push_back(makeStatus("motor_driver", diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor fault"));

  while (!callback_active_)
  {
    publishDiagnostics(statuses);
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

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

  while (ms_node_->count_subscribers("/diagnostics") == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  std::vector<diagnostic_msgs::msg::DiagnosticStatus> statuses;
  statuses.push_back(makeStatus("battery", diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery nominal"));
  statuses.push_back(makeStatus("motor_driver", diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor fault"));

  while (!callback_active_)
  {
    publishDiagnostics(statuses);
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  ASSERT_EQ(data_json_["statuses"].size(), 1u);
  EXPECT_EQ(data_json_["statuses"][0]["name"], "battery");
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
