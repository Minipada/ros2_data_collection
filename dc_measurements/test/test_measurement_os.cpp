#include <gtest/gtest.h>
#include <sys/utsname.h>

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
    os_callback_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  unsigned int cpu_count_;
  std::string kernel_;
  float memory_;

public:
  bool os_callback_{ false };
};

TEST_F(MeasurementOSTest, OSDataCorrect)
{
  ms_node_->declare_parameter("os.plugin", std::string("dc_measurements/OS"));
  ms_node_->declare_parameter("os.group_key", std::string("os"));
  ms_node_->declare_parameter("os.topic_output", std::string("/dc/measurement/os"));

  startLifecycleNode();

  while (!os_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_EQ(cpu_count_, std::thread::hardware_concurrency());
  EXPECT_GT(memory_, 0.0);
  EXPECT_LE(memory_, 100.0);

  utsname result;
  uname(&result);

  EXPECT_EQ(kernel_, result.release);
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
