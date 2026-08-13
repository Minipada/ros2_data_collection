#include <gtest/gtest.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

// Map::collect() shells out to `ros2 run nav2_map_server map_saver_cli`, which is not a
// dependency of this package and is not guaranteed to be installed, and even if it is, it needs
// a live /map topic to save from. This suite only exercises the deterministic, environment-
// independent path: no /map data means map_saver_cli (if present) times out or (if absent) the
// shell command fails immediately, either way collect() falls through to an empty Record --
// covering that the plugin degrades gracefully rather than crashing or hanging the node.
class MeasurementMapTest : public ::testing::Test
{
protected:
  MeasurementMapTest()
  {
    SetUp();
  }

  ~MeasurementMapTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "map" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/map", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementMapTest::mapDataCallback, this, std::placeholders::_1));
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

  void mapDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  nlohmann::json data_json_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementMapTest, NoMapDataYieldsEmptyRecordWithoutCrashing)
{
  // Keep saves under the test's scratch dir rather than the real $HOME default, and keep
  // map_saver_cli's own wait for /map short in case it is actually installed.
  ms_node_->declare_parameter("save_local_base_path", std::string("/tmp/dc_measurement_map_test"));
  ms_node_->declare_parameter("map.plugin", std::string("dc_measurements/Map"));
  ms_node_->declare_parameter("map.group_key", std::string("map"));
  ms_node_->declare_parameter("map.topic_output", std::string("/dc/measurement/map"));
  ms_node_->declare_parameter("map.topic", std::string("/test/map"));
  ms_node_->declare_parameter("map.save_map_timeout", 1.0);

  startLifecycleNode();

  while (!callback_active_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

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
