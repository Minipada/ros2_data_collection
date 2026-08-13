#include <gtest/gtest.h>

#include <algorithm>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

class MeasurementNetworkTest : public ::testing::Test
{
protected:
  MeasurementNetworkTest()
  {
    SetUp();
  }

  ~MeasurementNetworkTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "network" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/network", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementNetworkTest::networkDataCallback, this, std::placeholders::_1));
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

  void networkDataCallback(const dc_interfaces::msg::StringStamped& msg)
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

// Whether the ICMP ping itself succeeds depends on raw-socket (CAP_NET_RAW/root) privileges and
// real network access, both of which vary by CI/sandbox environment -- so this only asserts on
// the parts of the Record that are deterministic on any host: the network interface list (every
// Linux host has at least loopback) and the shape/types of the ping fields, not their values.
TEST_F(MeasurementNetworkTest, ReportsLocalInterfacesAndPingShape)
{
  ms_node_->declare_parameter("network.plugin", std::string("dc_measurements/Network"));
  ms_node_->declare_parameter("network.group_key", std::string("network"));
  ms_node_->declare_parameter("network.topic_output", std::string("/dc/measurement/network"));
  // 127.0.0.1 rather than the 8.8.8.8 default so this doesn't depend on outbound internet access.
  ms_node_->declare_parameter("network.ping_address", std::string("127.0.0.1"));

  startLifecycleNode();

  while (!callback_active_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  ASSERT_TRUE(data_json_["interfaces"].is_array());
  EXPECT_NE(std::find(data_json_["interfaces"].begin(), data_json_["interfaces"].end(), "lo"),
            data_json_["interfaces"].end());
  ASSERT_TRUE(data_json_["ping"].is_number_integer());
  ASSERT_TRUE(data_json_["online"].is_boolean());
}

TEST_F(MeasurementNetworkTest, InvalidPingAddressDisablesTheMeasurement)
{
  ms_node_->declare_parameter("network.plugin", std::string("dc_measurements/Network"));
  ms_node_->declare_parameter("network.group_key", std::string("network"));
  ms_node_->declare_parameter("network.topic_output", std::string("/dc/measurement/network"));
  ms_node_->declare_parameter("network.ping_address", std::string("not-an-ip-address"));
  ms_node_->declare_parameter("network.polling_interval", 50);

  startLifecycleNode();

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  while ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time)).count() <
         150)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_FALSE(callback_active_);
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
