#include <gtest/gtest.h>

#include "dc_measurements/measurement_server.hpp"

class MeasurementServerTest : public ::testing::Test
{
protected:
  MeasurementServerTest()
  {
    SetUp();
  }

  ~MeasurementServerTest()
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "os" });
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

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
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
