#include <gtest/gtest.h>

#include <chrono>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MeasurementGateConditionTest : public ::testing::Test
{
protected:
  MeasurementGateConditionTest()
  {
    SetUp();
  }

  ~MeasurementGateConditionTest() override
  {
  }

  void SetUp() override
  {
    // condition_plugins is declared by the constructor itself, so it must be supplied as a
    // parameter override rather than via ms_node_->declare_parameter afterwards.
    auto options = rclcpp::NodeOptions().parameter_overrides(
        { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "moving" }) });
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(options, std::vector<std::string>{ "dummy" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/dummy", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementGateConditionTest::dummyDataCallback, this, std::placeholders::_1));
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

// Acceptance criterion: "condition never becomes true" -- with the gate condition never
// satisfied, nothing is ever published, including the init_collect Record normally fired on
// activation.
TEST_F(MeasurementGateConditionTest, ConditionNeverBecomingTrueNeverPublishesAnything)
{
  ms_node_->declare_parameter("dummy.init_collect", true);

  startLifecycleNode();

  spinFor(polling_interval_ * 5);

  EXPECT_EQ(dummy_callback_count_, 0);
}

// Acceptance criterion: latching semantics -- once armed by the gate condition becoming true,
// collection proceeds normally and keeps proceeding even after the condition becomes false
// again (no re-arming).
TEST_F(MeasurementGateConditionTest, ArmsOnceThenLatchesOpenEvenAfterConditionBecomesFalseAgain)
{
  ms_node_->declare_parameter("dummy.init_collect", false);

  startLifecycleNode();

  // Before the robot ever moves, no measurement is published.
  spinFor(polling_interval_ * 3);
  EXPECT_EQ(dummy_callback_count_, 0);

  // Arm the gate: one Odometry message above the speed threshold flips Moving to active.
  publishOdom(1.0);
  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  EXPECT_GE(dummy_callback_count_, 1);

  // Stop moving again: two below-threshold messages flip Moving back to inactive.
  publishOdom(0.0);
  publishOdom(0.0);
  spinFor(polling_interval_);

  int count_after_arming = dummy_callback_count_;
  spinFor(polling_interval_ * 3);

  // Collection keeps happening on every subsequent tick even though the underlying condition
  // is false again -- the gate never re-closes.
  EXPECT_GT(dummy_callback_count_, count_after_arming);
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
