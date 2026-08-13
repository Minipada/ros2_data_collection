#include <gtest/gtest.h>

#include <boost/asio.hpp>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

class MeasurementTCPHealthTest : public ::testing::Test
{
protected:
  MeasurementTCPHealthTest()
  {
    SetUp();
  }

  ~MeasurementTCPHealthTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "tcp_health" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/tcp_health", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementTCPHealthTest::tcpHealthDataCallback, this, std::placeholders::_1));
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

  void tcpHealthDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
  }

  // Asks the OS for a free ephemeral port by binding to port 0.
  static unsigned short reserveFreePort()
  {
    boost::asio::io_service svc;
    boost::asio::ip::tcp::acceptor acceptor(svc);
    acceptor.open(boost::asio::ip::tcp::v4());
    acceptor.bind({ boost::asio::ip::tcp::v4(), 0 });
    return acceptor.local_endpoint().port();
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  nlohmann::json data_json_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementTCPHealthTest, ActiveWhenPortIsListening)
{
  boost::asio::io_service svc;
  boost::asio::ip::tcp::acceptor acceptor(svc);
  acceptor.open(boost::asio::ip::tcp::v4());
  acceptor.bind({ boost::asio::ip::tcp::v4(), 0 });
  acceptor.listen();
  unsigned short port = acceptor.local_endpoint().port();

  ms_node_->declare_parameter("tcp_health.plugin", std::string("dc_measurements/TCPHealth"));
  ms_node_->declare_parameter("tcp_health.group_key", std::string("tcp_health"));
  ms_node_->declare_parameter("tcp_health.topic_output", std::string("/dc/measurement/tcp_health"));
  ms_node_->declare_parameter("tcp_health.name", std::string("test-service"));
  ms_node_->declare_parameter("tcp_health.port", static_cast<int>(port));

  startLifecycleNode();

  while (!callback_active_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_EQ(data_json_["port"].get<int>(), port);
  EXPECT_EQ(data_json_["server_name"].get<std::string>(), "test-service");
  EXPECT_TRUE(data_json_["active"].get<bool>());
}

TEST_F(MeasurementTCPHealthTest, InactiveWhenPortIsFree)
{
  unsigned short port = reserveFreePort();

  ms_node_->declare_parameter("tcp_health.plugin", std::string("dc_measurements/TCPHealth"));
  ms_node_->declare_parameter("tcp_health.group_key", std::string("tcp_health"));
  ms_node_->declare_parameter("tcp_health.topic_output", std::string("/dc/measurement/tcp_health"));
  ms_node_->declare_parameter("tcp_health.name", std::string("test-service"));
  ms_node_->declare_parameter("tcp_health.port", static_cast<int>(port));

  startLifecycleNode();

  while (!callback_active_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_FALSE(data_json_["active"].get<bool>());
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
