#include <gtest/gtest.h>
#include <tf2_ros/transform_broadcaster.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class MeasurementDistanceTraveledTest : public ::testing::Test
{
protected:
  MeasurementDistanceTraveledTest()
  {
    SetUp();
  }

  ~MeasurementDistanceTraveledTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "distance_traveled" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/distance_traveled", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementDistanceTraveledTest::distanceDataCallback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ms_node_);
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void startLifecycleNode()
  {
    ms_node_->declare_parameter("distance_traveled.plugin", std::string("dc_measurements/DistanceTraveled"));
    ms_node_->declare_parameter("distance_traveled.group_key", std::string("distance_traveled"));
    ms_node_->declare_parameter("distance_traveled.topic_output", std::string("/dc/measurement/distance_traveled"));
    ms_node_->declare_parameter("distance_traveled.polling_interval", 50);
    // DistanceTraveled::onConfigure() reads "transform_timeout" via the non-throwing
    // get_parameter(name, out) overload without ever declaring it (a known pre-existing bug,
    // documented in distance_traveled.cpp) -- if left undeclared, transform_timeout_ keeps
    // whatever garbage value happened to be on the stack. Declared explicitly here so this test
    // exercises real, defined behaviour instead of UB.
    ms_node_->declare_parameter("distance_traveled.transform_timeout", 0.5);
    ms_node_->configure();
    ms_node_->activate();
  }

  void distanceDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
  }

  void broadcastMapToBaseLink(double x, double y)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = ms_node_->get_clock()->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nlohmann::json data_json_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementDistanceTraveledTest, PublishesDistanceFromOriginOnFirstFix)
{
  startLifecycleNode();

  // last_x_/last_y_ start at (0, 0), so the first successful transform lookup reports the
  // straight-line distance from the origin. Keep re-broadcasting and resetting until a Record
  // carrying "distance_traveled" shows up (earlier collect() cycles may fire before the
  // transform is in the tf buffer, publishing an empty "{}" Record instead).
  bool got_distance = false;
  while (!got_distance)
  {
    broadcastMapToBaseLink(3.0, 4.0);
    callback_active_ = false;
    rclcpp::spin_some(ms_node_->get_node_base_interface());
    if (callback_active_ && data_json_.contains("distance_traveled"))
    {
      got_distance = true;
    }
  }

  // sqrt(3^2 + 4^2)
  EXPECT_NEAR(data_json_["distance_traveled"].get<double>(), 5.0, 1e-2);
}

TEST_F(MeasurementDistanceTraveledTest, NoTransformYieldsEmptyRecord)
{
  startLifecycleNode();

  while (!callback_active_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_FALSE(data_json_.contains("distance_traveled"));
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
