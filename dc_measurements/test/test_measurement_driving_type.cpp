#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

class MeasurementDrivingTypeTest : public ::testing::Test
{
protected:
  MeasurementDrivingTypeTest()
  {
    SetUp();
  }

  ~MeasurementDrivingTypeTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "driving_type" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/driving_type", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementDrivingTypeTest::dataCallback, this, std::placeholders::_1));
    mode_pub_ = ms_node_->create_publisher<std_msgs::msg::String>("/driving_mode_raw", rclcpp::SystemDefaultsQoS());
    autonomous_vel_pub_ =
        ms_node_->create_publisher<geometry_msgs::msg::Twist>("/autonomy/cmd_vel", rclcpp::SystemDefaultsQoS());
    teleop_vel_pub_ =
        ms_node_->create_publisher<geometry_msgs::msg::Twist>("/teleop/cmd_vel", rclcpp::SystemDefaultsQoS());
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("driving_type.plugin", std::string("dc_measurements/DrivingType"));
    ms_node_->declare_parameter("driving_type.group_key", std::string("driving_type"));
    ms_node_->declare_parameter("driving_type.topic_output", std::string("/dc/measurement/driving_type"));
    ms_node_->declare_parameter("driving_type.polling_interval", 50);
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  void dataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
  }

  void spinUntil(std::function<bool()> predicate, int max_iterations = 500)
  {
    for (int i = 0; i < max_iterations && !predicate(); ++i)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(predicate()) << "Condition not met within the timeout";
  }

  void spinUntilCallback()
  {
    callback_active_ = false;
    spinUntil([this] { return callback_active_; });
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr autonomous_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_vel_pub_;
  nlohmann::json data_json_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementDrivingTypeTest, DefaultsToUnknownBeforeAnyModeIsObserved)
{
  declareCommonParameters();

  startLifecycleNode();
  spinUntilCallback();

  EXPECT_EQ(data_json_["mode"], "unknown");
}

TEST_F(MeasurementDrivingTypeTest, ModeTopicMapsRawValueToConfiguredMode)
{
  declareCommonParameters();
  ms_node_->declare_parameter("driving_type.mode_topic", std::string("/driving_mode_raw"));
  ms_node_->declare_parameter("driving_type.value_mapping_from", std::vector<std::string>{ "0", "1" });
  ms_node_->declare_parameter("driving_type.value_mapping_to", std::vector<std::string>{ "manual", "autonomous" });
  ms_node_->declare_parameter("driving_type.init_collect", false);

  startLifecycleNode();

  while (ms_node_->count_subscribers("/driving_mode_raw") == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  std_msgs::msg::String raw;
  raw.data = "1";
  while (!callback_active_)
  {
    mode_pub_->publish(raw);
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_EQ(data_json_["mode"], "autonomous");
}

TEST_F(MeasurementDrivingTypeTest, ModeTopicIgnoresUnmappedRawValueAndKeepsCurrentMode)
{
  declareCommonParameters();
  ms_node_->declare_parameter("driving_type.mode_topic", std::string("/driving_mode_raw"));
  ms_node_->declare_parameter("driving_type.value_mapping_from", std::vector<std::string>{ "1" });
  ms_node_->declare_parameter("driving_type.value_mapping_to", std::vector<std::string>{ "autonomous" });
  ms_node_->declare_parameter("driving_type.init_collect", false);

  startLifecycleNode();

  while (ms_node_->count_subscribers("/driving_mode_raw") == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  std_msgs::msg::String known;
  known.data = "1";
  while (!callback_active_)
  {
    mode_pub_->publish(known);
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  ASSERT_EQ(data_json_["mode"], "autonomous");

  callback_active_ = false;
  std_msgs::msg::String unmapped;
  unmapped.data = "99";
  mode_pub_->publish(unmapped);
  spinFor(std::chrono::milliseconds(200));

  ASSERT_TRUE(callback_active_);
  EXPECT_EQ(data_json_["mode"], "autonomous");
}

TEST_F(MeasurementDrivingTypeTest, VelocitySourceInferenceReportsModeOfLastActiveSource)
{
  declareCommonParameters();
  ms_node_->declare_parameter("driving_type.velocity_topics",
                              std::vector<std::string>{ "/autonomy/cmd_vel", "/teleop/cmd_vel" });
  ms_node_->declare_parameter("driving_type.velocity_modes", std::vector<std::string>{ "autonomous", "teleop" });
  ms_node_->declare_parameter("driving_type.velocity_timeout_s", 5.0);
  ms_node_->declare_parameter("driving_type.init_collect", false);

  startLifecycleNode();

  while (ms_node_->count_subscribers("/teleop/cmd_vel") == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  geometry_msgs::msg::Twist twist;
  while (!callback_active_)
  {
    teleop_vel_pub_->publish(twist);
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_EQ(data_json_["mode"], "teleop");
}

TEST_F(MeasurementDrivingTypeTest, VelocitySourceFallsBackToUnknownAfterTimeout)
{
  declareCommonParameters();
  ms_node_->declare_parameter("driving_type.velocity_topics", std::vector<std::string>{ "/autonomy/cmd_vel" });
  ms_node_->declare_parameter("driving_type.velocity_modes", std::vector<std::string>{ "autonomous" });
  ms_node_->declare_parameter("driving_type.velocity_timeout_s", 0.1);
  ms_node_->declare_parameter("driving_type.init_collect", false);

  startLifecycleNode();

  while (ms_node_->count_subscribers("/autonomy/cmd_vel") == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  geometry_msgs::msg::Twist twist;
  while (!callback_active_)
  {
    autonomous_vel_pub_->publish(twist);
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  ASSERT_EQ(data_json_["mode"], "autonomous");

  // Let the configured staleness window elapse with no further publishes on the source.
  callback_active_ = false;
  spinFor(std::chrono::milliseconds(400));

  ASSERT_TRUE(callback_active_);
  EXPECT_EQ(data_json_["mode"], "unknown");
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
