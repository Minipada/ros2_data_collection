// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "measurement_test_bench.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MeasurementMovingTest : public MeasurementBench
{
protected:
  MeasurementMovingTest()
    // condition_plugins is declared by the constructor itself, so it must be supplied as a
    // parameter override rather than via ms_node_->declare_parameter afterwards.
    : MeasurementBench("dummy", rclcpp::NodeOptions().parameter_overrides(
                                    { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "moving" }) }))
  {
    odom_pub_ = ms_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));

    ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
    ms_node_->declare_parameter("dummy.record", std::string("{\"message\": \"hello\"}"));
    ms_node_->declare_parameter("dummy.polling_interval", polling_interval_);
    ms_node_->declare_parameter("dummy.if_all_conditions", std::vector<std::string>{ "moving" });
    ms_node_->declare_parameter("dummy.init_max_measurements", -1);
    ms_node_->declare_parameter("dummy.condition_max_measurements", 0);

    ms_node_->declare_parameter("moving.plugin", std::string("dc_conditions/Moving"));
    ms_node_->declare_parameter("moving.odom_topic", std::string("/odom"));
    ms_node_->declare_parameter("moving.speed_threshold", 0.2);
    // 3 consecutive above/below-threshold Odometry messages required to flip state -- large
    // enough to distinguish "not yet enough messages" from "flipped" in the tests below.
    ms_node_->declare_parameter("moving.count_hysteresis", 3);
  }

  // Counting publishes is the whole capture here: whether and how often collection happened.
  void onRecord(const std::string& measurement, const dc_interfaces::msg::StringStamped& msg) override
  {
    (void)measurement;
    (void)msg;
    callback_active_ = true;
    callback_count_++;
  }

  void publishOdom(double linear_x)
  {
    nav_msgs::msg::Odometry msg;
    msg.twist.twist.linear.x = linear_x;
    odom_pub_->publish(msg);
    // Give the subscription callback a chance to run before the next publish.
    spinFor(polling_interval_ / 5);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  int polling_interval_{ 50 };
};

// Acceptance criterion: fewer than count_hysteresis consecutive above-threshold Odometry
// messages -- the condition has not yet flipped, so collection is still suppressed.
TEST_F(MeasurementMovingTest, FewerThanCountHysteresisMessagesNeverActivates)
{
  startLifecycleNode();

  publishOdom(1.0);
  publishOdom(1.0);
  spinFor(polling_interval_ * 3);

  EXPECT_EQ(callback_count_, 0);
}

// Acceptance criterion: exactly count_hysteresis consecutive above-threshold Odometry messages
// flips Moving active, and collection proceeds.
TEST_F(MeasurementMovingTest, CountHysteresisConsecutiveMessagesActivatesCondition)
{
  startLifecycleNode();

  publishOdom(1.0);
  publishOdom(1.0);
  publishOdom(1.0);

  ASSERT_TRUE(spinUntil([this] { return callback_count_ > 0; })) << "the condition never activated";
  EXPECT_GE(callback_count_, 1);
}

// Acceptance criterion: unlike gate_condition's one-way latch, if_all_conditions is
// re-evaluated every poll -- once Moving flips back to inactive (count_hysteresis consecutive
// below-threshold messages), collection stops again.
TEST_F(MeasurementMovingTest, ReturnsToInactiveAfterCountHysteresisConsecutiveBelowThresholdMessages)
{
  startLifecycleNode();

  publishOdom(1.0);
  publishOdom(1.0);
  publishOdom(1.0);
  ASSERT_TRUE(spinUntil([this] { return callback_count_ > 0; })) << "the condition never activated";

  // moving_count_ is at +count_hysteresis (3) after activation; each below-threshold message
  // only decrements it by 1, so flipping back to inactive (moving_count_ <= -count_hysteresis)
  // needs the full swing across the hysteresis band, not just count_hysteresis messages.
  for (int i = 0; i < 8; ++i)
  {
    publishOdom(0.0);
  }
  // Let any in-flight collection triggered just before the flip drain out.
  spinFor(polling_interval_ * 2);
  int count_after_stopping = callback_count_;

  spinFor(polling_interval_ * 3);
  EXPECT_EQ(callback_count_, count_after_stopping);
}

DC_MEASUREMENT_TEST_MAIN()
