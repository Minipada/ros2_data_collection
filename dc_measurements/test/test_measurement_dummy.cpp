#include <gtest/gtest.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

class MeasurementDummyTest : public ::testing::Test
{
protected:
  MeasurementDummyTest()
  {
    SetUp();
  }

  ~MeasurementDummyTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "dummy" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/dummy", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementDummyTest::dummyDataCallback, this, std::placeholders::_1));
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
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    nlohmann::json data_json = nlohmann::json::parse(data_str);
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    dummy_message_ = data_json["message"].get<nlohmann::json::string_t>();
    dummy_callback_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  std::string dummy_message_;

public:
  bool dummy_callback_{ false };
};

TEST_F(MeasurementDummyTest, DummyDataCorrect)
{
  auto record = std::string("{\"message\": \"My message\"}");
  auto message = nlohmann::json::parse(record)["message"];
  ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
  ms_node_->declare_parameter("dummy.record", record);

  startLifecycleNode();

  while (!dummy_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_EQ(dummy_message_, message);
}

TEST_F(MeasurementDummyTest, DummyDataIncorrect)
{
  int polling_interval = 50;
  ms_node_->declare_parameter("dummy.plugin", std::string("dc_measurements/Dummy"));
  ms_node_->declare_parameter("dummy.topic_output", std::string("/dc/measurement/dummy"));
  ms_node_->declare_parameter("dummy.polling_interval", polling_interval);
  // Not a valid JSON
  ms_node_->declare_parameter("dummy.record", std::string("{\"message\":"));

  startLifecycleNode();

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  // Check no sample has been published in the polling interval time, because exception was
  // triggered and the message did not go through
  while ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time)).count() <
         polling_interval * 2)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_TRUE(dummy_message_.empty());
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
