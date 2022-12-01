#include <memory>

#include "dc_destinations/destination_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto destination_node = std::make_shared<destination_server::DestinationServer>();

  executor.add_node(destination_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
