// The dc_bridge ROS node (ADRs 0001/0003/0006, and ADR-0007 which reverts the Rust
// pilot of ADR-0004): wires the ROS-independent core (Forwarder, Supervisor, Readiness,
// ConfigRenderer) to topic subscriptions and a readiness service.
//
// Vector's own configuration is produced by dc_bridge::render (ADR-0003) from the
// `shipper`/`destinations` parameters; this node declares those parameters, expands the
// $HOME/$DC_PG_PASSWORD-style env references the config contract uses, hands the result
// to the Supervisor, subscribes to the union of every Records-Destination's `inputs`
// topics, and forwards each Record to Vector over Fluent Forward. Raw Vector snippets in
// `custom_config_files` (ADR-0003 passthrough) are collision-checked and `vector
// validate`d together with the rendered config so a bad snippet fails loudly at startup.
//
// `receives: files` Destinations (ADR-0005, the Uploader) are handled in Phase 2.
#ifndef DC_BRIDGE__BRIDGE_NODE_HPP_
#define DC_BRIDGE__BRIDGE_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <vector>

#include "dc_bridge/forwarder.hpp"
#include "dc_bridge/readiness.hpp"
#include "dc_bridge/render.hpp"
#include "dc_bridge/supervisor.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"

namespace dc_bridge
{

class BridgeNode : public rclcpp::Node
{
public:
  explicit BridgeNode(const rclcpp::NodeOptions& options);
  ~BridgeNode() override;

  /// Stops the supervised Vector process and the background prober. Idempotent; called
  /// on rclcpp shutdown (see main.cpp) and from the destructor, so Vector is never
  /// orphaned on a normal Ctrl-C / `ros2 launch` stop / SIGTERM.
  void stop();

private:
  void run_prober(std::string forward_host, std::uint16_t forward_port);

  std::shared_ptr<Supervisor> supervisor_;
  std::mutex supervisor_mutex_;
  Readiness readiness_;

  std::shared_ptr<Forwarder> forwarder_;
  std::mutex forwarder_mutex_;

  std::vector<rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr> subscriptions_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr readiness_service_;

  std::thread prober_thread_;
  std::atomic<bool> prober_stop_{ false };
  std::atomic<bool> stopped_{ false };
};

}  // namespace dc_bridge

#endif  // DC_BRIDGE__BRIDGE_NODE_HPP_
