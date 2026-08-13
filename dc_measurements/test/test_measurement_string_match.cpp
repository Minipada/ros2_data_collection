#include <gtest/gtest.h>

#include <chrono>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

// Exercises dc_conditions::StringMatch through the if_all_conditions gating path: given a fixed
// Record (the dummy Measurement's static "record" JSON) and a condition config (key/regex),
// assert whether collection is activated or suppressed.
class MeasurementStringMatchTest : public ::testing::Test
{
protected:
  MeasurementStringMatchTest()
  {
    SetUp();
  }

  ~MeasurementStringMatchTest() override
  {
  }

  void SetUp() override
  {
    auto options = rclcpp::NodeOptions().parameter_overrides(
        { rclcpp::Parameter("condition_plugins", std::vector<std::string>{ "str_match" }) });
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(options, std::vector<std::string>{ "dummy" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/dummy", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementStringMatchTest::dummyDataCallback, this, std::placeholders::_1));

    ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
    ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
    ms_node_->declare_parameter("dummy.polling_interval", polling_interval_);
    ms_node_->declare_parameter("dummy.if_all_conditions", std::vector<std::string>{ "str_match" });
    ms_node_->declare_parameter("dummy.init_max_measurements", -1);
    ms_node_->declare_parameter("dummy.condition_max_measurements", 0);

    ms_node_->declare_parameter("str_match.plugin", std::string("dc_conditions/StringMatch"));
    ms_node_->declare_parameter("str_match.key", std::string("status"));
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

// Acceptance criterion: Record's string value matches the configured regex -> activates.
TEST_F(MeasurementStringMatchTest, StringMatchingRegexActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"status\": \"ok-123\"}"));
  ms_node_->declare_parameter("str_match.regex", std::string("ok-[0-9]+"));

  startLifecycleNode();

  while (dummy_callback_count_ == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  EXPECT_GE(dummy_callback_count_, 1);
}

// Acceptance criterion: Record's string value does not match the configured regex -> never
// activates.
TEST_F(MeasurementStringMatchTest, StringNotMatchingRegexNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"status\": \"ok-123\"}"));
  ms_node_->declare_parameter("str_match.regex", std::string("error-[0-9]+"));

  startLifecycleNode();

  spinFor(polling_interval_ * 5);
  EXPECT_EQ(dummy_callback_count_, 0);
}

// Acceptance criterion: std::regex_match requires a *full* match, not a substring search -- a
// regex matching only part of the value never activates.
TEST_F(MeasurementStringMatchTest, PartialSubstringMatchNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"status\": \"ok-123\"}"));
  ms_node_->declare_parameter("str_match.regex", std::string("ok"));

  startLifecycleNode();

  spinFor(polling_interval_ * 5);
  EXPECT_EQ(dummy_callback_count_, 0);
}

// Acceptance criterion: the configured key is absent from the Record -> never activates.
TEST_F(MeasurementStringMatchTest, MissingKeyNeverActivates)
{
  ms_node_->declare_parameter("dummy.record", std::string("{\"other\": \"ok-123\"}"));
  ms_node_->declare_parameter("str_match.regex", std::string("ok-[0-9]+"));

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
