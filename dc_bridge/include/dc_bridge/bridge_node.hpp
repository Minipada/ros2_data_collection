// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// The dc_bridge ROS node (ADRs 0001/0003/0006/0007): wires the ROS-independent core
// (Forwarder, Supervisor, Readiness, ConfigRenderer) to topic subscriptions and a
// readiness service.
//
// Vector's own configuration is produced by dc_bridge::render (ADR-0003) from the
// `shipper`/`destinations` parameters; this node declares those parameters, expands the
// $HOME/$VAR-style env references the config contract uses, and atomically writes the
// result to `shipper.config_path`. Raw Vector snippets in `custom_config_files`
// (ADR-0003 passthrough) are collision-checked against the rendered config and merged
// into that same file (dc_bridge::merge_custom_config_files) — one self-contained file
// regardless of mode, since in unmanaged/split-deployment mode a snippet's own
// filesystem path is never wired into the separate Shipper container. In the default
// managed mode (`shipper.managed: true`) the merged config is also handed to the
// Supervisor, which locates, `vector validate`s, and spawns/supervises the vendored
// Vector binary, so a bad snippet fails loudly at startup. In unmanaged mode
// (`shipper.managed: false`, #440/#444) none of that runs: an orchestrator owns the
// Shipper's lifecycle instead, and the Bridge only renders/writes the config and
// connects. Either way the node subscribes to the union of every Records-Destination's
// `inputs` topics and forwards each Record to Vector over the shipper ingest protocol.
//
// `receives: files` Destinations (ADR-0005): the Bridge subscribes and writes upload
// intents to the durable queue; a separate dc_uploader process (#446) reads, uploads, and
// emits the resulting status Records — see docs/adr/0014-uploader-runs-as-its-own-process.md.
#ifndef DC_BRIDGE__BRIDGE_NODE_HPP_
#define DC_BRIDGE__BRIDGE_NODE_HPP_

#include <atomic>
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
#include "dc_bridge/record_dispatch.hpp"
#include "dc_bridge/render.hpp"
#include "dc_bridge/supervisor.hpp"
#include "dc_bridge/uploader/file_status_tag.hpp"
#include "dc_bridge/uploader/intent_queue.hpp"
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

  // The durable upload intent queue (ADR-0005/#265) — created only when a `receives:
  // files` destination is configured. Records on files-destination topics are enqueued
  // here so they survive a Bridge crash/restart. The Bridge only writes; a separate
  // dc_uploader process (#446) owns reading, replaying, uploading, and emitting status
  // Records — see docs/adr/0014-uploader-runs-as-its-own-process.md.
  std::unique_ptr<uploader::IntentQueue> intent_queue_;

  // Per-Record dispatch (enqueue vs. forward, per topic). Borrows intent_queue_, the
  // Forwarder and its mutex above; declared after them so it dies first.
  std::unique_ptr<RecordDispatcher> dispatcher_;

  // Raw / generic-subscription mode (#227) — created only when `raw.enabled` is true.
  // Its Records go out through the same Forwarder as the Measurement Records above,
  // under the `dc.raw.<topic>` Tag namespace routed to `raw.destination`.
  RawConfig raw_config_;
  RawStats raw_stats_;
  std::unique_ptr<RawSubscriptionManager> raw_manager_;
};

}  // namespace dc_bridge

#endif  // DC_BRIDGE__BRIDGE_NODE_HPP_
