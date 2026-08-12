// Generic-subscription ("raw") mode, rclcpp side (#227): discovers topics on the ROS
// graph, subscribes to them with `create_generic_subscription` — no compile-time
// knowledge of their types, no generated headers, nothing the Bridge has to be rebuilt
// against — turns each serialized message into JSON through the runtime introspection
// type support, and hands the result to a sink as a Record under the `dc.raw.<topic>`
// Tag.
//
// This is the same mechanism `ros2 bag record -a` and PlotJuggler use: two type support
// libraries are loaded per message type at run time, one to deserialize
// (`rosidl_typesupport_cpp`) and one to describe the fields
// (`rosidl_typesupport_introspection_cpp`).
//
// Why an introspection walk rather than a converter dependency: `dynmsg`
// (`dynamic_message_introspection`) is the obvious candidate, but it has no binary
// release in the ROS 2 Jazzy distribution — adopting it means vendoring a source
// dependency into the workspace — and it emits YAML, which would then have to be parsed
// back into the JSON the ingest protocol actually carries. The walk below is ~200 lines
// against a stable, core-ROS ABI, produces `nlohmann::json` directly, and keeps
// dc_bridge's dependency list unchanged apart from type support packages every ROS
// install already has.
#ifndef DC_BRIDGE__RAW_SUBSCRIPTIONS_HPP_
#define DC_BRIDGE__RAW_SUBSCRIPTIONS_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcpputils/shared_library.hpp>
#include <set>
#include <string>

#include "dc_bridge/raw_config.hpp"

namespace dc_bridge
{

/// Deserializes and JSON-encodes messages of one runtime-known type.
///
/// Field mapping: numbers become JSON numbers, `string` becomes a JSON string,
/// `wstring` is transcoded UTF-16→UTF-8, nested messages become nested objects, and
/// arrays/sequences (fixed, bounded or unbounded) become JSON arrays — including
/// `uint8[]` blobs, which are *not* base64'd: raw mode's volume controls
/// (`raw.max_message_size_bytes`, `raw.exclude_types`) exist to keep those out rather
/// than to compress them.
class RawMessageConverter
{
public:
  /// Loads both type support libraries for `type_name` (e.g. `dc_interfaces/msg/
  /// StringStamped`). Throws RawConfigError if the type cannot be resolved at run time —
  /// which is what happens for a message package that isn't on this Bridge's
  /// AMENT_PREFIX_PATH.
  explicit RawMessageConverter(const std::string& type_name);
  ~RawMessageConverter();

  RawMessageConverter(const RawMessageConverter&) = delete;
  RawMessageConverter& operator=(const RawMessageConverter&) = delete;

  /// Deserializes `serialized` and converts it to a JSON object. Throws
  /// std::runtime_error if deserialization fails (a truncated or mistyped payload).
  nlohmann::json to_json(const rclcpp::SerializedMessage& serialized) const;

  const std::string& type_name() const noexcept
  {
    return type_name_;
  }

private:
  std::string type_name_;
  std::shared_ptr<rcpputils::SharedLibrary> typesupport_library_;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library_;
  /// `rosidl_typesupport_introspection_cpp::MessageMembers*`, kept type-erased so this
  /// header doesn't drag the introspection headers into every translation unit.
  const void* members_{ nullptr };
  std::unique_ptr<rclcpp::SerializationBase> serializer_;
};

/// A Record's time, split the way the ingest protocol's EventTime carries it.
struct RawStamp
{
  std::uint64_t secs{ 0 };
  std::uint32_t nanos{ 0 };
};

/// The `header.stamp` of a converted message, when it has one. Any message starting with
/// a `std_msgs/msg/Header` — the majority of what a robot publishes — is then Recorded at
/// the time the data was *captured* rather than the time the Bridge saw it. Messages
/// without a header fall back to the Bridge's own clock.
std::optional<RawStamp> stamp_from_payload(const nlohmann::json& payload);

/// QoS that can actually receive what a topic's current publishers send.
///
/// A generic subscription is useless if it silently fails to match: a `best_effort`
/// publisher (every sensor driver) is invisible to a `reliable` subscription, and a
/// latched (`transient_local`) topic delivers nothing at all to a `volatile` one until
/// it next publishes. This applies the same heuristic `ros2 bag record` uses — best
/// effort if *any* publisher is best effort, transient local if *every* publisher is —
/// with `depth` for history. Falls back to a plain reliable/volatile profile when the
/// topic has no publishers yet.
rclcpp::QoS raw_qos_for_topic(rclcpp::Node& node, const std::string& topic, std::size_t depth);

/// Owns raw mode's discovery timer and its generic subscriptions.
class RawSubscriptionManager
{
public:
  /// Sink for one raw Record. Returns false if the Shipper refused it (backpressure, a
  /// dropped connection) so the manager can count the drop; raw Records are *dropped at
  /// the source* rather than queued, see doc/src/dc/raw_topics.md.
  using RecordSink = std::function<bool(const std::string& tag, const RawStamp& stamp, nlohmann::json payload)>;

  /// Builds the manager and validates the filter regexes (throws RawConfigError on a bad
  /// pattern). Does not touch the graph until scan()/start() runs.
  RawSubscriptionManager(rclcpp::Node* node, RawConfig config, RecordSink sink, RawStats* stats);

  /// One discovery pass: subscribes to every newly-appeared topic that passes the filter.
  /// Idempotent — topics already subscribed are left alone. Never throws; a type that
  /// can't be resolved is logged once and skipped.
  void scan();

  /// Runs an immediate scan and, when `rescan_interval_secs > 0`, arms the periodic
  /// re-scan that picks up topics appearing after startup.
  void start();

  std::size_t subscription_count() const;

private:
  void on_raw_message(const std::string& topic, std::shared_ptr<const rclcpp::SerializedMessage> message);

  struct Entry
  {
    std::shared_ptr<rclcpp::GenericSubscription> subscription;
    std::shared_ptr<RawMessageConverter> converter;
    std::string tag;
  };

  rclcpp::Node* node_;
  RawConfig config_;
  RawTopicFilter filter_;
  RecordSink sink_;
  RawStats* stats_;

  mutable std::mutex mutex_;
  RawRateLimiter limiter_;
  std::map<std::string, Entry> subscriptions_;
  /// Topics already reported as skipped, so a 5-second rescan doesn't reprint the same
  /// verdict for the same topic forever.
  std::set<std::string> reported_;
  rclcpp::TimerBase::SharedPtr rescan_timer_;
};

}  // namespace dc_bridge

#endif  // DC_BRIDGE__RAW_SUBSCRIPTIONS_HPP_
