#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class MeasurementPositionTest : public ::testing::Test
{
protected:
  MeasurementPositionTest()
  {
    SetUp();
  }

  ~MeasurementPositionTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "position" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/position", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementPositionTest::positionDataCallback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ms_node_);
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void startLifecycleNode()
  {
    ms_node_->declare_parameter("position.plugin", std::string("dc_measurements/Position"));
    ms_node_->declare_parameter("position.group_key", std::string("position"));
    ms_node_->declare_parameter("position.topic_output", std::string("/dc/measurement/position"));
    ms_node_->declare_parameter("position.polling_interval", 50);
    ms_node_->configure();
    ms_node_->activate();
  }

  void positionDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
  }

  void broadcastMapToBaseLink(double x, double y, double yaw)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = ms_node_->get_clock()->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nlohmann::json data_json_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementPositionTest, PublishesPoseFromTransform)
{
  startLifecycleNode();

  // The very first collect() cycles may fire before the transform below is in the tf buffer,
  // yielding an empty Record ("{}"), which still gets published; keep broadcasting and
  // resetting until a Record carrying "x" actually shows up.
  const double yaw = 1.5707963267948966;  // pi / 2
  bool got_position = false;
  while (!got_position)
  {
    broadcastMapToBaseLink(2.0, 3.0, yaw);
    callback_active_ = false;
    rclcpp::spin_some(ms_node_->get_node_base_interface());
    if (callback_active_ && data_json_.contains("x"))
    {
      got_position = true;
    }
  }

  EXPECT_NEAR(data_json_["x"].get<double>(), 2.0, 1e-3);
  EXPECT_NEAR(data_json_["y"].get<double>(), 3.0, 1e-3);
  EXPECT_NEAR(data_json_["yaw"].get<double>(), yaw, 1e-3);
}

TEST_F(MeasurementPositionTest, NoTransformYieldsEmptyRecord)
{
  startLifecycleNode();

  while (!callback_active_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_FALSE(data_json_.contains("x"));
  EXPECT_TRUE(data_json_.empty());
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
