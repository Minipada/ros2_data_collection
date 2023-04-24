#include <gtest/gtest.h>
#include <sys/utsname.h>

#include <thread>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
class MeasurementServerTest : public ::testing::Test
{
protected:
  MeasurementServerTest()
  {
    SetUp();
  }

  ~MeasurementServerTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "os" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/os", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementServerTest::osDataCallback, this, std::placeholders::_1));
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
    // std::string yaml_str = dc_interfaces::msg::to_yaml(msg);
    // YAML::Node yaml_node = YAML::Load(yaml_str);
    // nlohmann::json data_json = dc_util::tojson::detail::yaml2json(yaml_node);

    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    nlohmann::json data_json = nlohmann::json::parse(data_str);
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    cpu_count_ = data_json["cpus"];
    kernel_ = data_json["kernel"];
    os_callback_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  unsigned int cpu_count_;
  std::string kernel_;

public:
  bool os_callback_{ false };
};

TEST_F(MeasurementServerTest, ParametersSaved)
{
  ms_node_->declare_parameter("os.plugin", std::string("dc_measurements/OS"));
  ms_node_->declare_parameter("os.group_key", std::string("os"));

  startLifecycleNode();

  std::vector<std::string> ms_plugins_desired = { "os" };
  std::vector<std::string> ms_types_desired = { "dc_measurements/OS" };
  std::vector<std::string> ms_group_key_desired = { "os" };

  EXPECT_EQ(ms_plugins_desired, ms_node_->getMeasurementPlugins());
  EXPECT_EQ(ms_types_desired, ms_node_->getMeasurementTypes());
  EXPECT_EQ(ms_group_key_desired, ms_node_->getMeasurementGroupKeys());
}

TEST_F(MeasurementServerTest, OSDataCorrect)
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
