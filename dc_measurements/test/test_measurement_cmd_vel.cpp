#include <gtest/gtest.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MeasurementCmdVelTest : public ::testing::Test
{
protected:
  MeasurementCmdVelTest()
  {
    SetUp();
  }

  ~MeasurementCmdVelTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "cmd_vel" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/cmd_vel", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementCmdVelTest::cmdVelDataCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = ms_node_->create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel", rclcpp::SystemDefaultsQoS());
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

  void cmdVelDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  nlohmann::json data_json_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementCmdVelTest, PublishesTwistWithComputedSpeed)
{
  ms_node_->declare_parameter("cmd_vel.plugin", std::string("dc_measurements/CmdVel"));
  ms_node_->declare_parameter("cmd_vel.group_key", std::string("cmd_vel"));
  ms_node_->declare_parameter("cmd_vel.topic_output", std::string("/dc/measurement/cmd_vel"));
  ms_node_->declare_parameter("cmd_vel.topic", std::string("/test/cmd_vel"));

  startLifecycleNode();

  while (ms_node_->count_subscribers("/test/cmd_vel") == 0)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 3.0;
  twist.linear.y = 4.0;
  twist.angular.z = 1.5;
  while (!callback_active_)
  {
    cmd_vel_pub_->publish(twist);
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_DOUBLE_EQ(data_json_["linear"]["x"].get<double>(), 3.0);
  EXPECT_DOUBLE_EQ(data_json_["linear"]["y"].get<double>(), 4.0);
  EXPECT_DOUBLE_EQ(data_json_["angular"]["z"].get<double>(), 1.5);
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
