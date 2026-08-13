#include <gtest/gtest.h>

#include <chrono>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

// dc_conditions::ListStringEqual is not in the original #158-#171 checklist but ships without a
// test either -- #255 calls it out explicitly as worth adding while sweeping the epic.
//
// Exercises it through the if_all_conditions gating path: given a fixed Record (the dummy
// Measurement's static "record" JSON) and a condition config (key/value/order_matters), assert
// whether collection is activated or suppressed.
class MeasurementListStringEqualTest : public ::testing::Test
{
protected:
  MeasurementListStringEqualTest()
  {
    SetUp();
  }

  ~MeasurementListStringEqualTest() override
  {
  }

  void SetUp() override
  {
    auto options = rclcpp::NodeOptions().parameter_overrides(
        { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "list_str_eq" }) });
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(options, std::vector<std::string>{ "dummy" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/dummy", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementListStringEqualTest::dummyDataCallback, this, std::placeholders::_1));

    ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
    ms_node_->declare_parameter("dummy.polling_interval", polling_interval_);
    ms_node_->declare_parameter("dummy.record", std::string("{\"tags\": [\"a\", \"b\", \"c\"]}"));
    ms_node_->declare_parameter("dummy.if_all_conditions", std::vector<std::string>{ "list_str_eq" });
    ms_node_->declare_parameter("dummy.init_max_measurements", -1);
    ms_node_->declare_parameter("dummy.condition_max_measurements", 0);

    ms_node_->declare_parameter("list_str_eq.plugin", std::string("dc_conditions/ListStringEqual"));
    ms_node_->declare_parameter("list_str_eq.key", std::string("tags"));
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
TEST_F(MeasurementListStringEqualTest, ExactOrderedMatchActivates)
{
  ms_node_->declare_parameter("list_str_eq.value", std::vector<std::string>{ "a", "b", "c" });

  startLifecycleNode();

  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  EXPECT_GE(dummy_callback_count_, 1);
}

// Acceptance criterion: same elements, different order, order_matters (default true) -> the
// exact-order comparison fails and never activates.
TEST_F(MeasurementListStringEqualTest, SameElementsDifferentOrderNeverActivatesWhenOrderMattersTrue)
{
  ms_node_->declare_parameter("list_str_eq.value", std::vector<std::string>{ "c", "a", "b" });

  startLifecycleNode();

  spinFor(polling_interval_ * 5);
  EXPECT_EQ(dummy_callback_count_, 0);
}

// Acceptance criterion: same elements, different order, order_matters=false -> the sorted
// comparison succeeds and activates.
TEST_F(MeasurementListStringEqualTest, SameElementsDifferentOrderActivatesWhenOrderMattersFalse)
{
  ms_node_->declare_parameter("list_str_eq.value", std::vector<std::string>{ "c", "a", "b" });
  ms_node_->declare_parameter("list_str_eq.order_matters", false);

  startLifecycleNode();

  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  EXPECT_GE(dummy_callback_count_, 1);
}

// Acceptance criterion: genuinely different elements (not just reordered), order_matters=false
// -> never activates even after sorting both sides. (ListStringEqual's reset branch already uses
// `!=` correctly, unlike the ListIntegerEqual/ListDoubleEqual bug fixed alongside this test.)
TEST_F(MeasurementListStringEqualTest, DifferentElementsNeverActivateWhenOrderMattersFalse)
{
  ms_node_->declare_parameter("list_str_eq.value", std::vector<std::string>{ "x", "y", "z" });
  ms_node_->declare_parameter("list_str_eq.order_matters", false);

  startLifecycleNode();

  spinFor(polling_interval_ * 5);
  EXPECT_EQ(dummy_callback_count_, 0);
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
