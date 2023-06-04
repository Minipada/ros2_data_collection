#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "dc_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleNodeTest : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleNodeTest() : rclcpp_lifecycle::LifecycleNode("lifecycle_node_test")
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Configured!");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Activated!");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Deactivated!");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Cleanup!");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Shutdown!");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State& /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is encountered an error!");
    return CallbackReturn::SUCCESS;
  }
};

class LifecycleClientTestFixture
{
public:
  LifecycleClientTestFixture()
  {
    lf_node_ = std::make_shared<LifecycleNodeTest>();
    lf_thread_ = std::make_unique<nav2_util::NodeThread>(lf_node_->get_node_base_interface());
  }

private:
  std::shared_ptr<LifecycleNodeTest> lf_node_;
  std::unique_ptr<nav2_util::NodeThread> lf_thread_;
};

TEST(LifecycleClientTest, BasicTest)
{
  LifecycleClientTestFixture fix;
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_service_client");
  dc_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test", node);
  EXPECT_EQ(dc_lifecycle_manager::SystemStatus::TIMEOUT, client.is_active(std::chrono::nanoseconds(1000)));
  EXPECT_TRUE(client.startup());
  EXPECT_EQ(dc_lifecycle_manager::SystemStatus::ACTIVE, client.is_active(std::chrono::nanoseconds(1000000000)));
  EXPECT_EQ(dc_lifecycle_manager::SystemStatus::ACTIVE, client.is_active());
  EXPECT_TRUE(client.pause());
  EXPECT_EQ(dc_lifecycle_manager::SystemStatus::INACTIVE, client.is_active(std::chrono::nanoseconds(1000000000)));
  EXPECT_TRUE(client.resume());
  EXPECT_TRUE(client.reset());
  EXPECT_TRUE(client.shutdown());
}

TEST(LifecycleClientTest, WithoutFixture)
{
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_service_client");
  dc_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test", node);
  EXPECT_EQ(dc_lifecycle_manager::SystemStatus::TIMEOUT, client.is_active(std::chrono::nanoseconds(1000)));
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
