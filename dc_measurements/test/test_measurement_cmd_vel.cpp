// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "geometry_msgs/msg/twist.hpp"
#include "measurement_test_bench.hpp"

class MeasurementCmdVelTest : public MeasurementBench
{
protected:
  MeasurementCmdVelTest() : MeasurementBench("cmd_vel")
  {
    cmd_vel_pub_ = ms_node_->create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel", rclcpp::SystemDefaultsQoS());
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

TEST_F(MeasurementCmdVelTest, PublishesTwistWithComputedSpeed)
{
  ms_node_->declare_parameter("cmd_vel.plugin", std::string("dc_measurements/CmdVel"));
  ms_node_->declare_parameter("cmd_vel.group_key", std::string("cmd_vel"));
  ms_node_->declare_parameter("cmd_vel.topic_output", std::string("/dc/measurement/cmd_vel"));
  ms_node_->declare_parameter("cmd_vel.topic", std::string("/test/cmd_vel"));

  startLifecycleNode();
  waitForSubscriber("/test/cmd_vel");

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 3.0;
  twist.linear.y = 4.0;
  twist.angular.z = 1.5;
  ASSERT_TRUE(spinUntil([&] {
    cmd_vel_pub_->publish(twist);
    return callback_active_;
  })) << "no Record ever arrived";

  EXPECT_DOUBLE_EQ(data_json_["linear"]["x"].get<double>(), 3.0);
  EXPECT_DOUBLE_EQ(data_json_["linear"]["y"].get<double>(), 4.0);
  EXPECT_DOUBLE_EQ(data_json_["angular"]["z"].get<double>(), 1.5);
  // sqrt(3^2 + 4^2) -- the plugin only factors linear x/y into "computed".
  EXPECT_NEAR(data_json_["computed"].get<double>(), 5.0, 1e-9);
}

DC_MEASUREMENT_TEST_MAIN()
