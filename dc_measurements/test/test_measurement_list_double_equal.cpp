#include <gtest/gtest.h>

#include <chrono>

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Exercises dc_conditions::ListDoubleEqual through the if_all_conditions gating path: given a
// fixed Record (the dummy Measurement's static "record" JSON) and a condition config
// (key/value/order_matters), assert whether collection is activated or suppressed.
class MeasurementListDoubleEqualTest : public ::testing::Test
{
protected:
  MeasurementListDoubleEqualTest()
  {
    SetUp();
  }

  ~MeasurementListDoubleEqualTest() override
  {
  }

  void SetUp() override
  {
    auto options = rclcpp::NodeOptions().parameter_overrides(
        { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "list_double_eq" }) });
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(options, std::vector<std::string>{ "dummy" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/dummy", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementListDoubleEqualTest::dummyDataCallback, this, std::placeholders::_1));

    ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
    ms_node_->declare_parameter("dummy.polling_interval", polling_interval_);
    ms_node_->declare_parameter("dummy.record", std::string("{\"values\": [1.1, 2.2, 3.3]}"));
    ms_node_->declare_parameter("dummy.if_all_conditions", std::vector<std::string>{ "list_double_eq" });
    ms_node_->declare_parameter("dummy.init_max_measurements", -1);
    ms_node_->declare_parameter("dummy.condition_max_measurements", 0);

    ms_node_->declare_parameter("list_double_eq.plugin", std::string("dc_conditions/ListDoubleEqual"));
    ms_node_->declare_parameter("list_double_eq.key", std::string("values"));
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
  int polling_interval_{ 50 };

public:
  int dummy_callback_count_{ 0 };
};

// Acceptance criterion: Record's array exactly matches the configured value, same order ->
// activates (order_matters defaults to true).
TEST_F(MeasurementListDoubleEqualTest, ExactOrderedMatchActivates)
{
  ms_node_->declare_parameter("list_double_eq.value", std::vector<double>{ 1.1, 2.2, 3.3 });

  startLifecycleNode();

  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  EXPECT_GE(dummy_callback_count_, 1);
}

// Acceptance criterion: same elements, different order, order_matters (default true) -> the
// exact-order comparison fails and never activates.
TEST_F(MeasurementListDoubleEqualTest, SameElementsDifferentOrderNeverActivatesWhenOrderMattersTrue)
{
  ms_node_->declare_parameter("list_double_eq.value", std::vector<double>{ 3.3, 1.1, 2.2 });

  startLifecycleNode();

  spinFor(polling_interval_ * 5);
  EXPECT_EQ(dummy_callback_count_, 0);
}

// Acceptance criterion: same elements, different order, order_matters=false -> the sorted
// comparison succeeds and activates.
TEST_F(MeasurementListDoubleEqualTest, SameElementsDifferentOrderActivatesWhenOrderMattersFalse)
{
  ms_node_->declare_parameter("list_double_eq.value", std::vector<double>{ 3.3, 1.1, 2.2 });
  ms_node_->declare_parameter("list_double_eq.order_matters", false);

  startLifecycleNode();

  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  EXPECT_GE(dummy_callback_count_, 1);
}

// Regression test for a copy-paste typo in the order_matters=false branch of getState():
// `else if (!order_matters_ && data_double == value_)` duplicated the `if`'s condition instead
// of negating it (`data_double != value_`), so the "reset to inactive" branch was unreachable
// dead code. Once the condition matched once, it stayed stuck active_=true forever, even for
// Records that plainly no longer matched -- an undocumented bug (unlike gate_condition's
// intentional latch), confirmed absent from the sibling ListBoolEqual/ListStringEqual plugins,
// which use `!=` correctly. Loads the plugin directly (bypassing the Measurement/
// if_all_conditions harness used elsewhere in this file) since demonstrating the transition
// needs two different Records fed to the *same* Condition instance over time, which a Dummy
// Measurement's static "record" param can't produce.
TEST(ListDoubleEqualDirectTest, UnorderedMatchTransitionsBackToInactiveWhenListsDiverge)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("list_double_eq_direct_test_node");
  node->declare_parameter("list_double_eq.key", std::string("values"));
  node->declare_parameter("list_double_eq.value", std::vector<double>{ 1.1, 2.2, 3.3 });
  node->declare_parameter("list_double_eq.order_matters", false);

  pluginlib::ClassLoader<dc_core::Condition> loader("dc_core", "dc_core::Condition");
  auto condition = loader.createUniqueInstance("dc_conditions/ListDoubleEqual");
  condition->configure(node, "list_double_eq");
  condition->activate();

  dc_interfaces::msg::StringStamped matching_msg;
  matching_msg.data = "{\"values\": [3.3, 1.1, 2.2]}";  // same elements, different order
  EXPECT_TRUE(condition->getState(matching_msg));

  dc_interfaces::msg::StringStamped differing_msg;
  differing_msg.data = "{\"values\": [9.9, 9.9, 9.9]}";
  EXPECT_FALSE(condition->getState(differing_msg));
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
