#include <gtest/gtest.h>

#include <chrono>
#include <set>

#include "dc_interfaces/msg/flush_event.hpp"
#include "dc_triggers/trigger_broadcast_node.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TriggerEdgeTriggerTest : public ::testing::Test
{
protected:
  TriggerEdgeTriggerTest()
  {
    SetUp();
  }

  ~TriggerEdgeTriggerTest() override
  {
  }

  void SetUp() override
  {
    // condition_plugins is read by the constructor itself, so it must be supplied as a parameter
    // override rather than via tb_node_->declare_parameter afterwards.
    auto options = rclcpp::NodeOptions().parameter_overrides(
        { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "moving" }) });
    tb_node_ = std::make_shared<trigger_broadcast_node::TriggerBroadcastNode>(options);

    sub_flush_ = tb_node_->create_subscription<dc_interfaces::msg::FlushEvent>(
        "/dc/flush", rclcpp::SystemDefaultsQoS(),
        std::bind(&TriggerEdgeTriggerTest::flushEventCallback, this, std::placeholders::_1));
    odom_pub_ = tb_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));

    tb_node_->declare_parameter("moving.plugin", std::string("dc_conditions/Moving"));
    tb_node_->declare_parameter("moving.odom_topic", std::string("/odom"));
    // A single Odometry message flips the Condition, keeping the test deterministic and fast.
    tb_node_->declare_parameter("moving.count_hysteresis", 1);

    tb_node_->declare_parameter("trigger.plugin", std::string("dc_triggers/EdgeTrigger"));
    tb_node_->declare_parameter("trigger.if_all_conditions", std::vector<std::string>{ "moving" });
    tb_node_->declare_parameter("trigger.topic", std::string("/dc/flush"));
    tb_node_->declare_parameter("trigger.polling_interval", polling_interval_);
  }

  void TearDown() override
  {
    tb_node_->deactivate();
    tb_node_->cleanup();
  }

  void startLifecycleNode()
  {
    tb_node_->configure();
    tb_node_->activate();
  }

  void flushEventCallback(const dc_interfaces::msg::FlushEvent& msg)
  {
    flush_count_++;
    incident_ids_.insert(msg.incident_id);
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
      rclcpp::spin_some(tb_node_->get_node_base_interface());
    }
  }

  std::shared_ptr<trigger_broadcast_node::TriggerBroadcastNode> tb_node_;
  rclcpp::Subscription<dc_interfaces::msg::FlushEvent>::SharedPtr sub_flush_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  int polling_interval_{ 20 };

public:
  int flush_count_{ 0 };
  std::set<std::string> incident_ids_;
};

// Acceptance criterion: the underlying Condition never becoming true never fires the Trigger.
TEST_F(TriggerEdgeTriggerTest, ConditionNeverBecomingTrueNeverFires)
{
  startLifecycleNode();

  spinFor(polling_interval_ * 5);

  EXPECT_EQ(flush_count_, 0);
}

// Acceptance criterion: exactly one FlushEvent per false->true rising edge, not on sustained true.
TEST_F(TriggerEdgeTriggerTest, FiresOnceOnRisingEdgeNotOnSustainedTrue)
{
  startLifecycleNode();

  spinFor(polling_interval_ * 3);
  EXPECT_EQ(flush_count_, 0);

  // Arm the Condition: one Odometry message above the speed threshold flips Moving to active.
  publishOdom(1.0);
  while (flush_count_ == 0)
  {
    rclcpp::spin_some(tb_node_->get_node_base_interface());
  }
  EXPECT_EQ(flush_count_, 1);

  // The Condition stays active (sustained true) for several more poll cycles -- no extra fire.
  spinFor(polling_interval_ * 5);
  EXPECT_EQ(flush_count_, 1);
}

// Acceptance criterion: a second rising edge (after the Condition falls and rises again) fires
// again, each firing carrying a distinct incident_id.
TEST_F(TriggerEdgeTriggerTest, SecondRisingEdgeFiresAgainWithDistinctIncidentId)
{
  startLifecycleNode();

  publishOdom(1.0);
  while (flush_count_ == 0)
  {
    rclcpp::spin_some(tb_node_->get_node_base_interface());
  }
  EXPECT_EQ(flush_count_, 1);

  // Stop moving: two below-threshold messages flip Moving back to inactive.
  publishOdom(0.0);
  publishOdom(0.0);
  spinFor(polling_interval_ * 3);
  EXPECT_EQ(flush_count_, 1);

  // Move again: the two below-threshold messages above left moving_count_ at -1, so it takes
  // two above-threshold messages (not one) to cross +count_hysteresis_ and re-fire the rising
  // edge -- the counter is symmetric, unlike gate_condition's one-shot latch.
  publishOdom(1.0);
  publishOdom(1.0);
  while (flush_count_ == 1)
  {
    rclcpp::spin_some(tb_node_->get_node_base_interface());
  }
  EXPECT_EQ(flush_count_, 2);
  EXPECT_EQ(incident_ids_.size(), 2u);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return all_successful;
}
