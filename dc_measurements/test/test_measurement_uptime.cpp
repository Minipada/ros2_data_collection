#include <gtest/gtest.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
class MeasurementUptimeTest : public ::testing::Test
{
protected:
  MeasurementUptimeTest()
  {
    SetUp();
  }

  ~MeasurementUptimeTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "uptime" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/uptime", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementUptimeTest::uptimeDataCallback, this, std::placeholders::_1));
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

  void uptimeDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    nlohmann::json data_json = nlohmann::json::parse(data_str);
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    uptime_ = data_json["time"].get<nlohmann::json::number_unsigned_t>();
    uptime_callback_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  unsigned int uptime_;

public:
  bool uptime_callback_{ false };
};

TEST_F(MeasurementUptimeTest, UptimeDataCorrect)
{
  ms_node_->declare_parameter("uptime.plugin", std::string("dc_measurements/Uptime"));
  ms_node_->declare_parameter("uptime.group_key", std::string("uptime"));
  ms_node_->declare_parameter("uptime.topic_output", std::string("/dc/measurement/uptime"));

  startLifecycleNode();

  while (!uptime_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_TRUE(uptime_ > 0);
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
