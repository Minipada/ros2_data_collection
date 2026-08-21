// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_OPEN_RMF_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_OPEN_RMF_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "dc_common/websocket_json_client.hpp"
#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/mission_record_json.hpp"
#include "dc_measurements/pending_record_queue.hpp"
#include "dc_measurements/plugins/measurements/mission_open_rmf_core.hpp"
#include "dc_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_measurements
{

/**
 * @class dc_measurements::MissionOpenRmf
 * @brief The Open-RMF adapter of the Mission Measurement (#396) for `TaskState` (#391): emits
 * `mission_start`/`mission_end` Records in #305/#387's Record contract from Open-RMF's own
 * per-task lifecycle, with `mission_id` sourced from `booking.id` and `mission_type` from
 * `category`.
 *
 * Unlike the nav2 adapters (#387/#388/#389), Open-RMF exposes no ROS action/topic for task
 * lifecycle -- `TaskState` is consumed as a stream of JSON documents over a plain `ws://`
 * connection (`dc_common::WebSocketJsonClient`, #390) against a `websocket_url` that bridges the
 * `rmf-web` API server's task-state feed into one `TaskState` JSON object per text frame. The
 * websocket handshake/reconnect protocol `rmf-web` actually speaks on its own endpoints (Socket.IO
 * framing) is out of scope for both #390 (transport-only, plain JSON text frames) and this issue;
 * a deployment points `websocket_url` at whatever small bridge/proxy re-emits that feed as plain
 * JSON frames.
 *
 * A passive observer, like the nav2 adapters: it never dispatches, cancels, or otherwise mutates
 * any Open-RMF task, only watches the state stream. See MissionOpenRmfCore for the status-mapping
 * logic (including the `skipped`/`blocked`/`error` judgment calls) and
 * `doc/src/dc/measurements/mission_open_rmf.md` for what this Measurement deliberately does not
 * represent (phase-level detail, interruptions).
 */
class MissionOpenRmf : public dc_measurements::Measurement
{
public:
  MissionOpenRmf();
  ~MissionOpenRmf() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  /// Fires on WebSocketJsonClient's own IO thread, not the node's executor -- everything it
  /// touches (core_, pending_records_) is guarded by mutex_.
  void handleMessage(const nlohmann::json& message);
  void emit(json data, const rclcpp::Time& stamp);

  std::string websocket_url_;
  std::unique_ptr<dc_common::WebSocketJsonClient> client_;

  mutable std::mutex mutex_;
  MissionOpenRmfCore core_;
  // Records wait here for a poll to carry them out, one per poll, so they travel the same publish
  // path (Conditions, buffering, Group) as every other Record -- same shape as
  // MissionNav2ThroughPoses::pending_records_.
  PendingRecordQueue pending_records_;

protected:
  void onConfigure() override;
  void onCleanup() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_OPEN_RMF_HPP_
