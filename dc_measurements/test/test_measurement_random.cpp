#include <gtest/gtest.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

class MeasurementRandomTest : public ::testing::Test
{
protected:
  MeasurementRandomTest()
  {
    SetUp();
  }

  ~MeasurementRandomTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "random" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/random", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementRandomTest::randomDataCallback, this, std::placeholders::_1));
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("random.plugin", std::string("dc_measurements/Random"));
    ms_node_->declare_parameter("random.group_key", std::string("random"));
    ms_node_->declare_parameter("random.topic_output", std::string("/dc/measurement/random"));
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  void randomDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    nlohmann::json data_json = nlohmann::json::parse(data_str);
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    value_ = data_json["value"].get<double>();
    random_callback_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  double value_;

public:
  bool random_callback_{ false };
};

TEST_F(MeasurementRandomTest, DefaultIntegerWithinRange)
{
  declareCommonParameters();

  startLifecycleNode();

  while (!random_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_GE(value_, 0);
  EXPECT_LE(value_, 100);
}

TEST_F(MeasurementRandomTest, DoubleWithinConfiguredRange)
{
  declareCommonParameters();
  ms_node_->declare_parameter("random.type", std::string("double"));
  ms_node_->declare_parameter("random.min", 5.0);
  ms_node_->declare_parameter("random.max", 6.0);

  startLifecycleNode();

  while (!random_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_GE(value_, 5.0);
  EXPECT_LE(value_, 6.0);
}

TEST_F(MeasurementRandomTest, SameSeedProducesSameSequence)
{
  declareCommonParameters();
  ms_node_->declare_parameter("random.min", 0.0);
  ms_node_->declare_parameter("random.max", 1000000.0);
  ms_node_->declare_parameter("random.seed", 42);

  startLifecycleNode();

  while (!random_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }
  double first_run_value = value_;

  // Cycle the same node back through configure/activate (parameters stay declared across
  // lifecycle transitions): the plugin is re-instantiated and re-seeded from the same
  // `seed` parameter, so its first emitted value must reproduce exactly.
  ms_node_->deactivate();
  ms_node_->cleanup();
  random_callback_ = false;
  startLifecycleNode();

  while (!random_callback_)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_EQ(first_run_value, value_);
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
