#include <gtest/gtest.h>

#include <chrono>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

// Exercises dc_conditions::SameAsPrevious through the if_all_conditions gating path: given a
// fixed Record (the dummy Measurement's static "record" JSON) and a condition config (no "keys"
// configured, so it compares the whole flattened Record rather than hashing specific file
// fields), assert whether collection is activated or suppressed.
//
// A Dummy Measurement's "record" param is static for the node's lifetime (read once in
// onConfigure(), never re-read per poll), so this fixture can only observe "the Record never
// changes" -- SameAsPrevious's active_=false transition when a *changed* Record arrives is not
// covered here; it would need a Record source whose content can vary between polls, which no
// existing Measurement plugin exposes as a live-updatable parameter.
class MeasurementSameAsPreviousTest : public ::testing::Test
{
protected:
  MeasurementSameAsPreviousTest()
  {
    SetUp();
  }

  ~MeasurementSameAsPreviousTest() override
  {
  }

  void SetUp() override
  {
    auto options = rclcpp::NodeOptions().parameter_overrides(
        { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "same_prev" }) });
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(options, std::vector<std::string>{ "dummy" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/dummy", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementSameAsPreviousTest::dummyDataCallback, this, std::placeholders::_1));

    ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
    ms_node_->declare_parameter("dummy.polling_interval", polling_interval_);
    ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.5}"));
    ms_node_->declare_parameter("dummy.if_all_conditions", std::vector<std::string>{ "same_prev" });
    ms_node_->declare_parameter("dummy.init_max_measurements", -1);
    ms_node_->declare_parameter("dummy.condition_max_measurements", 0);

    ms_node_->declare_parameter("same_prev.plugin", std::string("dc_conditions/SameAsPrevious"));
    ms_node_->declare_parameter("same_prev.keys", std::vector<std::string>());
    ms_node_->declare_parameter("same_prev.exclude", std::vector<std::string>());
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
  // Deliberately wider than the other test files here: the very first poll is guaranteed
  // suppressed (previous_json_ starts empty) while the second poll is guaranteed to activate
  // (the record never changes), so distinguishing "before poll #2" from "at/after poll #2" needs
  // a window with real margin instead of racing a tight timer.
  int polling_interval_{ 200 };

public:
  int dummy_callback_count_{ 0 };
};

// Acceptance criterion: the very first Record has nothing to compare against
// (previous_json_.empty()) -> the condition is inactive and collection is suppressed. Checked
// well inside the first polling_interval_ window, before a second poll could activate it.
TEST_F(MeasurementSameAsPreviousTest, FirstCollectionNeverActivates)
{
  startLifecycleNode();

  spinFor(polling_interval_ - 100);
  EXPECT_EQ(dummy_callback_count_, 0);
}

// Acceptance criterion: once a previous Record exists, an identical subsequent Record activates
// the condition -- and since the dummy Record never changes, it keeps activating from then on.
TEST_F(MeasurementSameAsPreviousTest, IdenticalSubsequentRecordsActivateFromSecondCollectionOnward)
{
  startLifecycleNode();

  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  int first_count = dummy_callback_count_;

  spinFor(polling_interval_ * 3);
  EXPECT_GT(dummy_callback_count_, first_count);
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
