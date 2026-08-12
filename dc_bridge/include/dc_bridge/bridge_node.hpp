// The dc_bridge ROS node (ADRs 0001/0003/0006/0007): wires the ROS-independent core
// (Forwarder, Supervisor, Readiness, ConfigRenderer) to topic subscriptions and a
// readiness service.
//
// Vector's own configuration is produced by dc_bridge::render (ADR-0003) from the
// `shipper`/`destinations` parameters; this node declares those parameters, expands the
// $HOME/$DC_PG_PASSWORD-style env references the config contract uses, hands the result
// to the Supervisor, subscribes to the union of every Records-Destination's `inputs`
// topics, and forwards each Record to Vector over the shipper ingest protocol. Raw
// Vector snippets in `custom_config_files` (ADR-0003 passthrough) are collision-checked
// and `vector validate`d together with the rendered config so a bad snippet fails loudly
// at startup.
//
// `receives: files` Destinations (ADR-0005, the Uploader) are handled in Phase 2.
#ifndef DC_BRIDGE__BRIDGE_NODE_HPP_
#define DC_BRIDGE__BRIDGE_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <vector>

#include "dc_bridge/forwarder.hpp"
#include "dc_bridge/raw_config.hpp"
#include "dc_bridge/raw_subscriptions.hpp"
#include "dc_bridge/readiness.hpp"
#include "dc_bridge/render.hpp"
#include "dc_bridge/supervisor.hpp"
#include "dc_bridge/uploader/intent_queue.hpp"
#include "dc_bridge/uploader/object_store.hpp"
#include "dc_bridge/uploader/retention.hpp"
#include "dc_bridge/uploader/uploader.hpp"
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
  // The Uploader worker (ADR-0005/#265): sweeps the durable intent queue oldest-first
  // (per-entry backoff on failure — one permanently-failing intent can't starve the
  // backlog), processes each Record, acks (unlinks) its intent on success, and emits
  // status Records through its own Forwarder connection under FILE_STATUS_TAG.
  void run_uploader_worker(std::string forward_host, std::uint16_t forward_port);

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

  // Uploader (ADR-0005) — created only when a `receives: files` destination is
  // configured. Records on files-destination topics are enqueued into the durable
  // intent_queue_ (#265); the worker thread replays/sweeps it, uploads Files, and emits
  // status Records.
  std::unique_ptr<uploader::Uploader> uploader_;
  std::unique_ptr<uploader::IntentQueue> intent_queue_;
  std::thread uploader_thread_;
  std::mutex uploader_wake_mutex_;
  std::condition_variable uploader_wake_cv_;
  bool upload_stop_{ false };

  // Files retention (#267) — a copy of the storages the Uploader was built with (kept
  // alongside it since the Uploader moves its own copy) so the worker thread can build
  // shed audit rows without needing an Uploader accessor; a no-op sweep when disabled
  // (the default) either way.
  uploader::RetentionConfig retention_config_;
  std::vector<Storage> files_storages_;

  // Raw / generic-subscription mode (#227) — created only when `raw.enabled` is true.
  // Its Records go out through the same Forwarder as the Measurement Records above,
  // under the `dc.raw.<topic>` Tag namespace routed to `raw.destination`.
  RawConfig raw_config_;
  RawStats raw_stats_;
  std::unique_ptr<RawSubscriptionManager> raw_manager_;
};

}  // namespace dc_bridge

#endif  // DC_BRIDGE__BRIDGE_NODE_HPP_
