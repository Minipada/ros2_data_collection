// dc_bridge node entry point. rclcpp handles SIGINT/SIGTERM and returns from spin();
// on_shutdown stops the supervised Vector before exit so it's never orphaned (the same
// guarantee the Rust node's ctrlc handler provided, but native to rclcpp here).
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "dc_bridge/bridge_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  int rc = 0;
  try
  {
    auto node = std::make_shared<dc_bridge::BridgeNode>(rclcpp::NodeOptions());
    // Stop Vector on any rclcpp shutdown path (Ctrl-C, SIGTERM, `ros2 launch` stop).
    rclcpp::on_shutdown([node]() { node->stop(); });
    rclcpp::spin(node);
    node->stop();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("dc_bridge"), "fatal: %s", e.what());
    rc = 1;
  }

  rclcpp::shutdown();
  return rc;
}
