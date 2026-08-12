// Generic-subscription ("raw") mode policy (#227): which discovered topics the Bridge
// subscribes to with no compile-time knowledge of their types, the Tag each one's
// Records carry, and the volume controls that stop a firehose topic from outrunning the
// Shipper.
//
// Deliberately ROS-free — the filter/tag/rate-limit decisions are pure functions of a
// topic name, a type name, a serialized size and a clock, so they are unit-tested with
// plain gtest (raw_config_test) and no ROS graph. The rclcpp side (discovery,
// `create_generic_subscription`, introspection→JSON) lives in raw_subscriptions.hpp,
// which is built into the node target only.
#ifndef DC_BRIDGE__RAW_CONFIG_HPP_
#define DC_BRIDGE__RAW_CONFIG_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace dc_bridge
{

/// Tag namespace every raw Record is published under: `dc.raw.<sanitised topic>`. The
/// trailing dot matters — the rendered Vector config routes the whole namespace with one
/// `starts_with(.tag, …)` branch (see route_output_for_tag_prefix), which is what lets
/// topics discovered *after* Vector started still reach their Destination.
inline constexpr const char* RAW_TAG_PREFIX = "dc.raw.";

/// Raw-mode configuration, as declared from the `raw.*` ROS parameters.
struct RawConfig
{
  bool enabled{ false };
  /// Name of the `receives: records` Destination raw Records are routed to. Required
  /// when enabled — a raw Record with nowhere to go is a config error, not a default.
  std::string destination;
  /// Topic-name regexes (ECMAScript, `regex_search` — unanchored unless you anchor
  /// them). A topic must match at least one to be subscribed.
  std::vector<std::string> include;
  /// Topic-name regexes that veto a match from `include`.
  std::vector<std::string> exclude;
  /// Message-*type* regexes (matched against e.g. `sensor_msgs/msg/Image`) that veto a
  /// topic regardless of its name — the type is the robust way to keep the high-rate
  /// sensor streams out (a camera topic is not reliably named `*image*`).
  std::vector<std::string> exclude_types;
  /// How often the topic graph is re-scanned so topics appearing after startup get
  /// picked up. 0 disables re-scanning (one scan at startup).
  double rescan_interval_secs{ 5.0 };
  /// Depth of each generic subscription's QoS history.
  std::size_t qos_depth{ 10 };
  /// Per-topic ceiling on how many messages are turned into Records each second.
  /// 0 = unlimited. This is raw mode's first line of volume defence (see
  /// doc/src/dc/raw_topics.md "Backpressure and volume").
  double max_rate_hz{ 10.0 };
  /// Serialized messages larger than this are dropped, not truncated. 0 = unlimited.
  std::uint64_t max_message_size_bytes{ 1048576 };
  /// Tag namespace; overridable, but the rendered route follows whatever is set here.
  std::string tag_prefix{ RAW_TAG_PREFIX };
};

/// Topic-name regexes excluded unless the operator overrides `raw.exclude`: ROS's own
/// infrastructure topics (`/rosout` would also feed the Bridge's own warnings back into
/// the pipeline) and DC's own Record topics, which already reach their Destinations as
/// Measurement Records — subscribing to them raw as well would ship everything twice
/// under two different Tags.
std::vector<std::string> default_raw_exclude();

/// Message types excluded unless the operator overrides `raw.exclude_types`: the
/// high-rate sensor streams (#227's third open question — answered "yes, excluded by
/// default"). One 640x480 `sensor_msgs/msg/Image` is ~900 kB of JSON numbers at 30 Hz;
/// nothing downstream of the Bridge is sized for that, and a user who genuinely wants it
/// can say so explicitly.
std::vector<std::string> default_raw_exclude_types();

/// Why a discovered topic was (not) subscribed. Reported once per topic at discovery
/// time, so an operator can see what the filters actually did.
enum class RawSkip
{
  Accepted,
  NotIncluded,
  Excluded,
  ExcludedType,
};

const char* to_string(RawSkip skip);

/// Thrown for a raw config that cannot be honoured (an uncompilable regex, an enabled
/// mode with no destination). Surfaces as a loud Bridge startup failure, like every
/// other config error (ADR-0003).
class RawConfigError : public std::runtime_error
{
public:
  explicit RawConfigError(const std::string& message) : std::runtime_error(message)
  {
  }
};

/// Compiled include/exclude policy. Construction validates every regex, so a typo in a
/// params file fails at startup rather than silently matching nothing.
class RawTopicFilter
{
public:
  explicit RawTopicFilter(const RawConfig& config);

  /// Full verdict for one discovered topic, including *why* it was skipped.
  RawSkip evaluate(const std::string& topic, const std::string& type) const;

  bool accepts(const std::string& topic, const std::string& type) const
  {
    return evaluate(topic, type) == RawSkip::Accepted;
  }

private:
  std::vector<std::regex> include_;
  std::vector<std::regex> exclude_;
  std::vector<std::regex> exclude_types_;
};

/// The Tag a raw topic's Records carry: `<prefix><topic with the leading `/` stripped and
/// every remaining `/` turned into `.`>`, with anything outside `[A-Za-z0-9_.]` mapped to
/// `_`. `/dc/e2e/synth/synth00` → `dc.raw.dc.e2e.synth.synth00`. Sanitising matters
/// because the Tag ends up in a VRL string literal and a Vector route name; ROS topic
/// tokens are already `[A-Za-z0-9_]`, so the mapping is a no-op in practice and a
/// safety net for anything a future ROS naming rule allows.
std::string raw_tag(const std::string& tag_prefix, const std::string& topic);

/// Per-topic decimation: at most one message per `1/max_rate_hz` seconds — specifically,
/// a message passes once that long has elapsed since the last one that passed. So each
/// Record is the newest message at the instant it is emitted (nothing is buffered and
/// released later); between emissions, the most recent Record is up to one window old.
///
/// Deliberately not a token bucket — a bucket lets a burst through all at once, which is
/// exactly the shape of traffic this exists to flatten. Dropping by time like this is
/// uniform: every window is represented, so the result is an unbiased lower-resolution
/// time series. (Contrast `max_message_size_bytes`, which drops by *content* and
/// therefore biases what survives — see doc/src/dc/raw_topics.md.)
///
/// Not thread-safe; the subscription manager owns the lock (raw callbacks can run on
/// several executor threads).
class RawRateLimiter
{
public:
  using Clock = std::chrono::steady_clock;

  explicit RawRateLimiter(double max_rate_hz);

  /// True if `topic` may emit a Record now (and records that it did).
  bool allow(const std::string& topic, Clock::time_point now);

  bool unlimited() const noexcept
  {
    return min_interval_ == Clock::duration::zero();
  }

private:
  Clock::duration min_interval_;
  std::unordered_map<std::string, Clock::time_point> last_pass_;
};

/// Raw-mode counters, surfaced through the Bridge's `~/ready` service so an operator can
/// see whether raw mode is shedding — the drop counters are the observable half of the
/// documented backpressure behaviour.
struct RawStats
{
  std::atomic<std::uint64_t> subscribed_topics{ 0 };
  std::atomic<std::uint64_t> forwarded{ 0 };
  std::atomic<std::uint64_t> dropped_rate{ 0 };
  std::atomic<std::uint64_t> dropped_oversize{ 0 };
  std::atomic<std::uint64_t> dropped_shipper{ 0 };
  std::atomic<std::uint64_t> dropped_undecodable{ 0 };

  /// One-line human summary for logs and the readiness service.
  std::string summary() const;
};

}  // namespace dc_bridge

#endif  // DC_BRIDGE__RAW_CONFIG_HPP_
