// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "geometry_msgs/msg/twist.hpp"
#include "measurement_test_bench.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MeasurementSpeedTest : public MeasurementBench
{
protected:
  MeasurementSpeedTest() : MeasurementBench("speed")
  {
    odom_pub_ = ms_node_->create_publisher<nav_msgs::msg::Odometry>("/test/odom", rclcpp::SystemDefaultsQoS());
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

TEST_F(MeasurementSpeedTest, PublishesTwistWithComputedSpeed)
{
  ms_node_->declare_parameter("speed.plugin", std::string("dc_measurements/Speed"));
  ms_node_->declare_parameter("speed.group_key", std::string("speed"));
  ms_node_->declare_parameter("speed.topic_output", std::string("/dc/measurement/speed"));
  ms_node_->declare_parameter("speed.odom_topic", std::string("/test/odom"));

  startLifecycleNode();
  waitForSubscriber("/test/odom");

  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = 3.0;
  odom.twist.twist.linear.y = 4.0;
  odom.twist.twist.angular.z = 0.5;
  ASSERT_TRUE(spinUntil([&] {
    odom_pub_->publish(odom);
    return callback_active_;
  })) << "no Record ever arrived";

  EXPECT_DOUBLE_EQ(data_json_["linear"]["x"].get<double>(), 3.0);
  EXPECT_DOUBLE_EQ(data_json_["linear"]["y"].get<double>(), 4.0);
  EXPECT_DOUBLE_EQ(data_json_["angular"]["z"].get<double>(), 0.5);
  // sqrt(3^2 + 4^2) -- the plugin only factors linear x/y into "computed".
  EXPECT_NEAR(data_json_["computed"].get<double>(), 5.0, 1e-9);
}

DC_MEASUREMENT_TEST_MAIN()
