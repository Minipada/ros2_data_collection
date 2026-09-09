// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "measurement_test_bench.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MeasurementGateConditionTest : public MeasurementBench
{
protected:
  MeasurementGateConditionTest()
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
    ms_node_->declare_parameter("dummy.gate_condition", std::string("moving"));

    ms_node_->declare_parameter("moving.plugin", std::string("dc_conditions/Moving"));
    ms_node_->declare_parameter("moving.odom_topic", std::string("/odom"));
    // A single Odometry message flips the Condition, keeping the test deterministic and fast.
    ms_node_->declare_parameter("moving.count_hysteresis", 1);
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
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  int polling_interval_{ 50 };
};

// Acceptance criterion: "condition never becomes true" -- with the gate condition never
// satisfied, nothing is ever published, including the init_collect Record normally fired on
// activation.
TEST_F(MeasurementGateConditionTest, ConditionNeverBecomingTrueNeverPublishesAnything)
{
  ms_node_->declare_parameter("dummy.init_collect", true);

  startLifecycleNode();

  spinFor(polling_interval_ * 5);

  EXPECT_EQ(callback_count_, 0);
}

// Acceptance criterion: latching semantics -- once armed by the gate condition becoming true,
// collection proceeds normally and keeps proceeding even after the condition becomes false
// again (no re-arming).
TEST_F(MeasurementGateConditionTest, ArmsOnceThenLatchesOpenEvenAfterConditionBecomesFalseAgain)
{
  ms_node_->declare_parameter("dummy.init_collect", false);

  startLifecycleNode();
  waitForSubscriber("/odom");

  // Before the robot ever moves, no measurement is published.
  spinFor(polling_interval_ * 3);
  EXPECT_EQ(callback_count_, 0);

  // Arm the gate: one Odometry message above the speed threshold flips Moving to active.
  publishOdom(1.0);
  ASSERT_TRUE(spinUntil([this] { return callback_count_ > 0; })) << "collection never started";
  EXPECT_GE(callback_count_, 1);

  // Stop moving again: two below-threshold messages flip Moving back to inactive.
  publishOdom(0.0);
  publishOdom(0.0);
  spinFor(polling_interval_);

  int count_after_arming = callback_count_;
  spinFor(polling_interval_ * 3);

  // Collection keeps happening on every subsequent tick even though the underlying condition
  // is false again -- the gate never re-closes.
  EXPECT_GT(callback_count_, count_after_arming);
}

DC_MEASUREMENT_TEST_MAIN()
