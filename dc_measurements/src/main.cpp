#include <memory>

#include "dc_measurements/measurement_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto measurement_node = std::make_shared<measurement_server::MeasurementServer>();

  executor.add_node(measurement_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
