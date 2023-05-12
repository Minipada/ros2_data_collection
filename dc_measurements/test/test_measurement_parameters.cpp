#include <gtest/gtest.h>
#include <sys/utsname.h>

#include <chrono>
#include <thread>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

class MeasurementOSTest : public ::testing::Test
{
protected:
  MeasurementOSTest()
  {
    SetUp();
  }

  ~MeasurementOSTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "os" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/os", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementOSTest::osDataCallback, this, std::placeholders::_1));
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

  void osDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    nlohmann::json data_json = nlohmann::json::parse(data_str);
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    cpu_count_ = data_json["cpus"];
    kernel_ = data_json["kernel"];
    memory_ = data_json["memory"];
    callback_active_ = true;
    count_measurement_callback_++;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  unsigned int cpu_count_;
  std::string kernel_;
  float memory_;

public:
  bool callback_active_{ false };
  int count_measurement_callback_{ 0 };
};

TEST_F(MeasurementOSTest, PollingIntervalOneMeasurementOnePercentError)
{
  int polling_interval = 30;
  int count_measurement = 100;
  float error = 0.01;
  nav2_util::declare_parameter_if_not_declared(ms_node_, "os.plugin", rclcpp::ParameterValue("dc_measurements/OS"));
  nav2_util::declare_parameter_if_not_declared(ms_node_, "os.group_key", rclcpp::ParameterValue("os"));
  nav2_util::declare_parameter_if_not_declared(ms_node_, "os.topic_output",
                                               rclcpp::ParameterValue("/dc/measurement/os"));
  nav2_util::declare_parameter_if_not_declared(ms_node_, "os.polling_interval",
                                               rclcpp::ParameterValue(polling_interval));
  nav2_util::declare_parameter_if_not_declared(ms_node_, "os.init_collect", rclcpp::ParameterValue(false));

  startLifecycleNode();

  // Verify parameters are set properly
  std::vector<std::string> ms_plugins_desired = { "os" };
  std::vector<std::string> ms_types_desired = { "dc_measurements/OS" };
  std::vector<std::string> ms_group_key_desired = { "os" };
  std::vector<std::string> ms_topic_output_desired = { "/dc/measurement/os" };
  std::vector<int> ms_polling_interval_desired = { polling_interval };
  std::vector<bool> ms_init_collect_desired = { false };

  EXPECT_EQ(ms_plugins_desired, ms_node_->getMeasurementPlugins());
  EXPECT_EQ(ms_types_desired, ms_node_->getMeasurementTypes());
  EXPECT_EQ(ms_group_key_desired, ms_node_->getMeasurementGroupKeys());
  EXPECT_EQ(ms_topic_output_desired, ms_node_->getMeasurementTopicOutput());
  EXPECT_EQ(ms_polling_interval_desired, ms_node_->getMeasurementPollingInterval());
  EXPECT_EQ(ms_init_collect_desired, ms_node_->getMeasurementInitCollect());

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

  // Verify that in a certain amount of time, only a certain amount of samples are collected
  while ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time)).count() <
         polling_interval * (count_measurement + error))
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_EQ(count_measurement_callback_, count_measurement);
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
