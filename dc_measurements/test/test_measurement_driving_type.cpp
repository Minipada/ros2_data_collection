// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "measurement_test_bench.hpp"
#include "std_msgs/msg/string.hpp"

class MeasurementDrivingTypeTest : public MeasurementBench
{
protected:
  MeasurementDrivingTypeTest() : MeasurementBench("driving_type")
  {
    mode_pub_ = ms_node_->create_publisher<std_msgs::msg::String>("/driving_mode_raw", rclcpp::SystemDefaultsQoS());
    autonomous_vel_pub_ =
        ms_node_->create_publisher<geometry_msgs::msg::Twist>("/autonomy/cmd_vel", rclcpp::SystemDefaultsQoS());
    teleop_vel_pub_ =
        ms_node_->create_publisher<geometry_msgs::msg::Twist>("/teleop/cmd_vel", rclcpp::SystemDefaultsQoS());
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("driving_type.plugin", std::string("dc_measurements/DrivingType"));
    ms_node_->declare_parameter("driving_type.group_key", std::string("driving_type"));
    ms_node_->declare_parameter("driving_type.topic_output", std::string("/dc/measurement/driving_type"));
    ms_node_->declare_parameter("driving_type.polling_interval", 50);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr autonomous_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_vel_pub_;
};

TEST_F(MeasurementDrivingTypeTest, DefaultsToUnknownBeforeAnyModeIsObserved)
{
  declareCommonParameters();

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["mode"], "unknown");
}

TEST_F(MeasurementDrivingTypeTest, ModeTopicMapsRawValueToConfiguredMode)
{
  declareCommonParameters();
  ms_node_->declare_parameter("driving_type.mode_topic", std::string("/driving_mode_raw"));
  ms_node_->declare_parameter("driving_type.value_mapping_from", std::vector<std::string>{ "0", "1" });
  ms_node_->declare_parameter("driving_type.value_mapping_to", std::vector<std::string>{ "manual", "autonomous" });
  ms_node_->declare_parameter("driving_type.init_collect", false);

  startLifecycleNode();
  waitForSubscriber("/driving_mode_raw");

  std_msgs::msg::String raw;
  raw.data = "1";
  ASSERT_TRUE(spinUntil([&] {
    mode_pub_->publish(raw);
    return callback_active_;
  })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["mode"], "autonomous");
}

TEST_F(MeasurementDrivingTypeTest, ModeTopicIgnoresUnmappedRawValueAndKeepsCurrentMode)
{
  declareCommonParameters();
  ms_node_->declare_parameter("driving_type.mode_topic", std::string("/driving_mode_raw"));
  ms_node_->declare_parameter("driving_type.value_mapping_from", std::vector<std::string>{ "1" });
  ms_node_->declare_parameter("driving_type.value_mapping_to", std::vector<std::string>{ "autonomous" });
  ms_node_->declare_parameter("driving_type.init_collect", false);

  startLifecycleNode();
  waitForSubscriber("/driving_mode_raw");

  std_msgs::msg::String known;
  known.data = "1";
  ASSERT_TRUE(spinUntil([&] {
    mode_pub_->publish(known);
    return callback_active_;
  })) << "no Record ever arrived";
  ASSERT_EQ(data_json_["mode"], "autonomous");

  callback_active_ = false;
  std_msgs::msg::String unmapped;
  unmapped.data = "99";
  mode_pub_->publish(unmapped);
  spinFor(200);

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";
  EXPECT_EQ(data_json_["mode"], "autonomous");
}

TEST_F(MeasurementDrivingTypeTest, VelocitySourceInferenceReportsModeOfLastActiveSource)
{
  declareCommonParameters();
  ms_node_->declare_parameter("driving_type.velocity_topics",
                              std::vector<std::string>{ "/autonomy/cmd_vel", "/teleop/cmd_vel" });
  ms_node_->declare_parameter("driving_type.velocity_modes", std::vector<std::string>{ "autonomous", "teleop" });
  ms_node_->declare_parameter("driving_type.velocity_timeout_s", 5.0);
  ms_node_->declare_parameter("driving_type.init_collect", false);

  startLifecycleNode();
  waitForSubscriber("/teleop/cmd_vel");

  geometry_msgs::msg::Twist twist;
  ASSERT_TRUE(spinUntil([&] {
    teleop_vel_pub_->publish(twist);
    return callback_active_;
  })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["mode"], "teleop");
}

TEST_F(MeasurementDrivingTypeTest, VelocitySourceFallsBackToUnknownAfterTimeout)
{
  declareCommonParameters();
  ms_node_->declare_parameter("driving_type.velocity_topics", std::vector<std::string>{ "/autonomy/cmd_vel" });
  ms_node_->declare_parameter("driving_type.velocity_modes", std::vector<std::string>{ "autonomous" });
  ms_node_->declare_parameter("driving_type.velocity_timeout_s", 0.1);
  ms_node_->declare_parameter("driving_type.init_collect", false);

  startLifecycleNode();
  waitForSubscriber("/autonomy/cmd_vel");

  geometry_msgs::msg::Twist twist;
  ASSERT_TRUE(spinUntil([&] {
    autonomous_vel_pub_->publish(twist);
    return callback_active_;
  })) << "no Record ever arrived";
  ASSERT_EQ(data_json_["mode"], "autonomous");

  // Let the configured staleness window elapse with no further publishes on the source.
  callback_active_ = false;
  spinFor(400);

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";
  EXPECT_EQ(data_json_["mode"], "unknown");
}

DC_MEASUREMENT_TEST_MAIN()
