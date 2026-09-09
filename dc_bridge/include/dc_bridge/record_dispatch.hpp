// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Per-Record dispatch, ROS-free: decides, per topic, whether a Record becomes a durable
// upload intent for the Uploader, is forwarded to the Shipper, or both.
#ifndef DC_BRIDGE__RECORD_DISPATCH_HPP_
#define DC_BRIDGE__RECORD_DISPATCH_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "dc_bridge/forwarder.hpp"
#include "dc_bridge/uploader/intent_queue.hpp"

namespace dc_bridge
{

/// One Record arriving on a subscribed topic, as the ROS subscription hands it over: the
/// topic, the header stamp, and the payload bytes verbatim. No rclcpp/dc_interfaces types
/// — the Bridge node's subscription callback is the only thing that sees StringStamped.
struct IncomingRecord
{
  std::string topic;
  /// The ROS header stamp as the header carries it: `sec` is signed, and a negative value
  /// is clamped to 0 when the forwarded Record is built.
  std::int32_t stamp_secs{ 0 };
  std::uint32_t stamp_nanos{ 0 };
  std::string data;
};

/// The Bridge's subscription-callback body: both hand-offs a Record can get, and the
/// files-vs-records decision behind them. Accepts the upload intent queue and the
/// Forwarder behind their existing interfaces; it creates neither.
class RecordDispatcher
{
public:
  /// The topics the Destinations' `inputs` name, split by what they receive.
  struct Topics
  {
    std::vector<std::string> records;  ///< inputs of `receives: records` Destinations
    std::vector<std::string> files;    ///< inputs of `receives: files` Destinations
  };

  /// Dependencies, owned by the caller (the Bridge node) — pointers, so a dependency the
  /// caller doesn't have can stay unset.
  struct Deps
  {
    /// The durable upload intent queue; must be set whenever Topics::files is non-empty.
    uploader::IntentQueue* intent_queue{ nullptr };
    Forwarder* forwarder{ nullptr };
    /// The Bridge's Forwarder lock, shared with the prober thread's poll() and raw mode's
    /// send() — the Forwarder itself is not thread-safe.
    std::mutex* forwarder_mutex{ nullptr };
    /// Called when a forward fails. ROS-independent, so the Bridge node wires this to
    /// RCLCPP_WARN (same contract as ForwarderConfig::on_warning).
    std::function<void(const std::string&)> on_warning;
  };

  RecordDispatcher(Topics topics, Deps deps);

  /// Every topic both lists name, deduped — what the Bridge subscribes to, once each.
  const std::vector<std::string>& subscribed_topics() const
  {
    return subscribed_topics_;
  }

  /// Enqueues the intent (durable on disk before this returns), then forwards the Record.
  /// A failed forward is reported through `Deps::on_warning`, never thrown: one
  /// unreachable Shipper must not take down Record collection.
  void dispatch(const IncomingRecord& incoming) const;

  /// The Record's payload from `StringStamped.data`: parsed when the bytes are JSON,
  /// wrapped as a JSON string when they are not — never dropped. A non-object JSON value
  /// stays what it is, so the Forwarder's own pack_record_map still wraps it in
  /// {"message": ...} on the wire.
  static nlohmann::json parse_payload(const std::string& data);

private:
  struct Routing
  {
    std::string tag;
    bool enqueue_intent{ false };
    bool forward{ false };
  };

  std::map<std::string, Routing> routing_by_topic_;
  std::vector<std::string> subscribed_topics_;
  Deps deps_;
};

}  // namespace dc_bridge

#endif  // DC_BRIDGE__RECORD_DISPATCH_HPP_
