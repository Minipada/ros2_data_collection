#include <gtest/gtest.h>

#include <chrono>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

// dc_conditions::Moving is unlike the other condition plugins covered in this directory: it
// doesn't derive its state from the gated Measurement's own Record at all -- getState() is the
// base class's default (just returns the internally tracked active_), and that internal state is
// instead driven asynchronously by Moving's own /odom subscription and a hysteresis counter
// (moving_count_). test_measurement_gate_condition.cpp already exercises Moving incidentally
// (count_hysteresis=1, a single message flips it) to cover gate_condition's latch semantics; this
// file is Moving's own dedicated coverage and focuses on the hysteresis counting itself, using
// if_all_conditions (re-evaluated every poll, no latch) so the on/off transitions are directly
// observable.
class MeasurementMovingTest : public ::testing::Test
{
protected:
  MeasurementMovingTest()
  {
    SetUp();
  }

  ~MeasurementMovingTest() override
  {
  }

  void SetUp() override
  {
    auto options = rclcpp::NodeOptions().parameter_overrides(
        { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "moving" }) });
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(options, std::vector<std::string>{ "dummy" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/dummy", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementMovingTest::dummyDataCallback, this, std::placeholders::_1));
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

  void dummyDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    (void)msg;
    dummy_callback_count_++;
  }

  void publishOdom(double linear_x)
  {
    nav_msgs::msg::Odometry msg;
    msg.twist.twist.linear.x = linear_x;
    odom_pub_->publish(msg);
    // Give the subscription callback a chance to run before the next publish.
    spinFor(polling_interval_ / 5);
  }

  void spinFor(int milliseconds)
  {
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    while (
        (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time)).count() <
        milliseconds)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
    }
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  int polling_interval_{ 50 };

public:
  int dummy_callback_count_{ 0 };
};

// Acceptance criterion: fewer than count_hysteresis consecutive above-threshold Odometry
// messages -- the condition has not yet flipped, so collection is still suppressed.
TEST_F(MeasurementMovingTest, FewerThanCountHysteresisMessagesNeverActivates)
{
  startLifecycleNode();

  publishOdom(1.0);
  publishOdom(1.0);
  spinFor(polling_interval_ * 3);

  EXPECT_EQ(dummy_callback_count_, 0);
}

// Acceptance criterion: exactly count_hysteresis consecutive above-threshold Odometry messages
// flips Moving active, and collection proceeds.
TEST_F(MeasurementMovingTest, CountHysteresisConsecutiveMessagesActivatesCondition)
{
  startLifecycleNode();

  publishOdom(1.0);
  publishOdom(1.0);
  publishOdom(1.0);

  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  EXPECT_GE(dummy_callback_count_, 1);
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
  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  // moving_count_ is at +count_hysteresis (3) after activation; each below-threshold message
  // only decrements it by 1, so flipping back to inactive (moving_count_ <= -count_hysteresis)
  // needs the full swing across the hysteresis band, not just count_hysteresis messages.
  for (int i = 0; i < 8; ++i)
  {
    publishOdom(0.0);
  }
  // Let any in-flight collection triggered just before the flip drain out.
  spinFor(polling_interval_ * 2);
  int count_after_stopping = dummy_callback_count_;

  spinFor(polling_interval_ * 3);
  EXPECT_EQ(dummy_callback_count_, count_after_stopping);
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
