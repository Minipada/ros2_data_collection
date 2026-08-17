// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <chrono>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

// Exercises dc_conditions::Exist through the if_all_conditions gating path: given a fixed Record
// (the dummy Measurement's static "record" JSON) and a condition config (key), assert whether
// collection is activated or suppressed.
class MeasurementExistTest : public ::testing::Test
{
protected:
  MeasurementExistTest()
  {
    SetUp();
  }

  ~MeasurementExistTest() override
  {
  }

  void SetUp() override
  {
    auto options = rclcpp::NodeOptions().parameter_overrides(
        { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "exist" }) });
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(options, std::vector<std::string>{ "dummy" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/dummy", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementExistTest::dummyDataCallback, this, std::placeholders::_1));

    ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
    ms_node_->declare_parameter("dummy.polling_interval", polling_interval_);
    ms_node_->declare_parameter("dummy.if_all_conditions", std::vector<std::string>{ "exist" });
    ms_node_->declare_parameter("dummy.init_max_measurements", -1);
    ms_node_->declare_parameter("dummy.condition_max_measurements", 0);

    ms_node_->declare_parameter("exist.plugin", std::string("dc_conditions/Exist"));
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

// Acceptance criterion: the configured key is present in the Record -> the condition activates.
TEST_F(MeasurementExistTest, KeyPresentInRecordActivatesCondition)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.5}"));
  ms_node_->declare_parameter("exist.key", std::string("level"));

  startLifecycleNode();

  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  int first_count = dummy_callback_count_;

  spinFor(polling_interval_ * 3);
  EXPECT_GT(dummy_callback_count_, first_count);
}

// Acceptance criterion: the configured key is absent from the Record -> the condition never
// activates and nothing is ever published.
TEST_F(MeasurementExistTest, KeyAbsentFromRecordNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"other\": 5.5}"));
  ms_node_->declare_parameter("exist.key", std::string("level"));

  startLifecycleNode();

  spinFor(polling_interval_ * 5);

  EXPECT_EQ(dummy_callback_count_, 0);
}

// Acceptance criterion: a nested key (found via the flattened "/parent/child" prefix match, not
// an exact top-level key) still counts as existing.
TEST_F(MeasurementExistTest, NestedKeyIsFoundAndActivatesCondition)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"parent\": {\"child\": 1}}"));
  ms_node_->declare_parameter("exist.key", std::string("parent"));

  startLifecycleNode();

  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  EXPECT_GE(dummy_callback_count_, 1);
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
