// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <chrono>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

// Exercises dc_conditions::IntegerEqual through the if_all_conditions gating path: given a fixed
// Record (the dummy Measurement's static "record" JSON) and a condition config (key/value),
// assert whether collection is activated or suppressed.
class MeasurementIntegerEqualTest : public ::testing::Test
{
protected:
  MeasurementIntegerEqualTest()
  {
    SetUp();
  }

  ~MeasurementIntegerEqualTest() override
  {
  }

  void SetUp() override
  {
    auto options = rclcpp::NodeOptions().parameter_overrides(
        { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "integer_eq" }) });
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(options, std::vector<std::string>{ "dummy" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/dummy", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementIntegerEqualTest::dummyDataCallback, this, std::placeholders::_1));

    ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
    ms_node_->declare_parameter("dummy.polling_interval", polling_interval_);
    ms_node_->declare_parameter("dummy.if_all_conditions", std::vector<std::string>{ "integer_eq" });
    ms_node_->declare_parameter("dummy.init_max_measurements", -1);
    ms_node_->declare_parameter("dummy.condition_max_measurements", 0);

    ms_node_->declare_parameter("integer_eq.plugin", std::string("dc_conditions/IntegerEqual"));
    ms_node_->declare_parameter("integer_eq.key", std::string("level"));
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

// Acceptance criterion: Record's key holds a JSON integer equal to the configured value ->
// activates and keeps activating on every poll.
TEST_F(MeasurementIntegerEqualTest, RecordValueMatchingConfiguredValueActivatesCondition)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("integer_eq.value", 5);

  startLifecycleNode();

  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  int first_count = dummy_callback_count_;

  spinFor(polling_interval_ * 3);
  EXPECT_GT(dummy_callback_count_, first_count);
}

// Acceptance criterion: Record's key holds a different integer -> never activates.
TEST_F(MeasurementIntegerEqualTest, RecordValueDifferingFromConfiguredValueNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5}"));
  ms_node_->declare_parameter("integer_eq.value", 3);

  startLifecycleNode();

  spinFor(polling_interval_ * 5);
  EXPECT_EQ(dummy_callback_count_, 0);
}

// Acceptance criterion: the configured key is absent from the Record -> never activates.
TEST_F(MeasurementIntegerEqualTest, MissingKeyInRecordNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"other\": 5}"));
  ms_node_->declare_parameter("integer_eq.value", 5);

  startLifecycleNode();

  spinFor(polling_interval_ * 5);
  EXPECT_EQ(dummy_callback_count_, 0);
}

// Acceptance criterion / gotcha (mirrors DoubleEqual's type-strictness): a Record field written
// as a float literal is numerically equal to the configured integer value but parses to
// json::value_t::number_float, not number_integer, so IntegerEqual treats it as "not an integer"
// and never activates.
TEST_F(MeasurementIntegerEqualTest, DoubleLiteralInRecordNeverMatchesDespiteNumericEquality)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"level\": 5.0}"));
  ms_node_->declare_parameter("integer_eq.value", 5);

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
