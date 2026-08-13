#include <gtest/gtest.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MeasurementSpeedTest : public ::testing::Test
{
protected:
  MeasurementSpeedTest()
  {
    SetUp();
  }

  ~MeasurementSpeedTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "speed" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/speed", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementSpeedTest::speedDataCallback, this, std::placeholders::_1));
    odom_pub_ = ms_node_->create_publisher<nav_msgs::msg::Odometry>("/test/odom", rclcpp::SystemDefaultsQoS());
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

  void speedDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  nlohmann::json data_json_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementSpeedTest, PublishesTwistWithComputedSpeed)
{
  ms_node_->declare_parameter("speed.plugin", std::string("dc_measurements/Speed"));
  ms_node_->declare_parameter("speed.group_key", std::string("speed"));
  ms_node_->declare_parameter("speed.topic_output", std::string("/dc/measurement/speed"));
  ms_node_->declare_parameter("speed.odom_topic", std::string("/test/odom"));

  startLifecycleNode();

  while (ms_node_->count_subscribers("/test/odom") == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = 3.0;
  odom.twist.twist.linear.y = 4.0;
  odom.twist.twist.angular.z = 0.5;
  while (!callback_active_)
  {
    odom_pub_->publish(odom);
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_DOUBLE_EQ(data_json_["linear"]["x"].get<double>(), 3.0);
  EXPECT_DOUBLE_EQ(data_json_["linear"]["y"].get<double>(), 4.0);
  EXPECT_DOUBLE_EQ(data_json_["angular"]["z"].get<double>(), 0.5);
  // sqrt(3^2 + 4^2) -- the plugin only factors linear x/y into "computed".
  EXPECT_NEAR(data_json_["computed"].get<double>(), 5.0, 1e-9);
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
