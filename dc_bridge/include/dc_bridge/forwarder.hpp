// Forwarder: send(record) hides shipper ingest protocol msgpack framing, socket
// lifecycle, reconnection, backpressure, and delivery confirmation behind one call.
#ifndef DC_BRIDGE__FORWARDER_HPP_
#define DC_BRIDGE__FORWARDER_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <msgpack.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>

namespace dc_bridge
{

/// A DC Record ready to be forwarded: a shipper ingest protocol tag, a Unix timestamp
/// split into whole seconds plus the nanoseconds within that second, and the Record's
/// JSON payload (StringStamped.data, already parsed).
///
/// The split mirrors the ingest protocol's **EventTime** extension (4 bytes of seconds,
/// 4 of nanoseconds), which is what the Forwarder emits. The protocol's older integer
/// time carries whole seconds only; sending that rounded every Record's timestamp to the
/// second, so a Measurement polling faster than 1 Hz produced Records that were
/// indistinguishable in time (#308).
struct Record
{
  std::string tag;
  std::uint64_t timestamp_secs;
  /// Nanoseconds within `timestamp_secs`; always < 1'000'000'000.
  std::uint32_t timestamp_nanos{ 0 };
  nlohmann::json payload;
};

struct ForwarderConfig
{
  std::string host;
  std::uint16_t port;
  std::chrono::milliseconds connect_timeout{ 2000 };
  std::chrono::milliseconds write_timeout{ 200 };
  /// A sent-but-unacked record is resent if no `ack` for it arrives within this window
  /// (in addition to being resent unconditionally after a reconnect).
  std::chrono::milliseconds ack_timeout{ 5000 };
  /// Bounds on the in-memory unacked window (#266): once either is exceeded, the oldest
  /// unacked record is dropped to make room. Sizing this is the lever for how much
  /// in-flight data survives a Bridge crash concurrent with a Vector outage (the
  /// documented double-failure this window does NOT cover) — it is not a substitute for
  /// `shipper.buffer_max_bytes`, which is Vector's own disk buffer for sink outages.
  std::size_t max_unacked_records{ 10000 };
  std::size_t max_unacked_bytes{ 16ULL * 1024 * 1024 };
  /// Called (at most once per `warn_interval`) when the unacked window drops records to
  /// stay within bounds. ROS-independent, so the Bridge node wires this to RCLCPP_WARN.
  std::function<void(const std::string&)> on_warning;
  std::chrono::milliseconds warn_interval{ 5000 };
};

/// Error categories a send() can fail with, so callers (and tests) can distinguish
/// backpressure (retryable, connection kept) from a dropped connection / connect
/// failure.
enum class ForwarderErrorKind
{
  Connect,       ///< could not establish the TCP connection
  Backpressure,  ///< peer stalled; write did not complete within write_timeout
  Io,            ///< io error mid-write; the connection was dropped
};

class ForwarderError : public std::runtime_error
{
public:
  ForwarderError(ForwarderErrorKind kind, const std::string& msg) : std::runtime_error(msg), kind_(kind)
  {
  }
  ForwarderErrorKind kind() const noexcept
  {
    return kind_;
  }

private:
  ForwarderErrorKind kind_;
};

/// Sends Records to the shipper ingest protocol peer (Vector's `fluent` source) over
/// TCP, with confirmed delivery (#266): each frame carries a unique `chunk` id (the
/// protocol's acknowledgement option); Vector's `ack` response for a chunk removes it
/// from an in-memory "unacked window" of sent-but-unconfirmed records. Anything still in
/// the window is resent after a reconnect (the peer may never have seen it) or after
/// ack_timeout elapses with no response (the ack may have been lost, not just delayed).
/// This makes delivery at-least-once: a record acked after the sender already resent it
/// (e.g. a reconnect racing a slow ack) can land twice — never zero times short of the
/// window itself overflowing (see max_unacked_records/max_unacked_bytes) or a
/// Bridge-crash-during-outage double failure (out of scope, see ADR-0002).
///
/// Reconnects lazily: a dead connection is dropped on write failure and re-established
/// on the next send()/poll(). A write that blocks past write_timeout is reported as
/// Backpressure without dropping the connection, since the peer may just be slow to
/// drain, not gone.
class Forwarder
{
public:
  explicit Forwarder(ForwarderConfig config);
  ~Forwarder();

  Forwarder(const Forwarder&) = delete;
  Forwarder& operator=(const Forwarder&) = delete;

  bool is_connected() const noexcept
  {
    return fd_ >= 0;
  }

  /// Sends one Record. Throws ForwarderError on failure (see ForwarderErrorKind). The
  /// record is added to the unacked window before the wire write is attempted, so it is
  /// still eligible for resend even if this particular write fails.
  void send(const Record& record);

  /// Opportunistically drains any pending acks and resends anything past ack_timeout (or
  /// resends everything if a reconnect just happened), without sending a new Record.
  /// Lets a caller (e.g. a periodic prober thread) keep the unacked window converging
  /// even while no new Records are being published. A no-op if there is nothing
  /// connected and nothing pending. Never throws.
  void poll() noexcept;

  /// Number of sent-but-unacked records currently held in the window. Exposed for tests
  /// and observability.
  std::size_t unacked_count() const noexcept
  {
    return window_.size();
  }

  /// Total frame bytes currently held in the unacked window.
  std::size_t unacked_bytes() const noexcept
  {
    return window_bytes_;
  }

  /// The shipper ingest protocol "Forward Mode" frame `[tag, [[time, record]], {"chunk":
  /// chunk_id}]` for `record`, as msgpack bytes. Exposed for the wire-format tests.
  static std::string frame(const Record& record, const std::string& chunk_id);

private:
  struct PendingRecord
  {
    std::string chunk_id;
    std::string frame_bytes;
    std::chrono::steady_clock::time_point sent_at;
  };

  void ensure_connected();
  void close_connection() noexcept;
  /// Writes `bytes` fully to the current connection. Throws ForwarderError (Backpressure
  /// or Io) on failure; Io drops the connection.
  void write_bytes(const std::string& bytes);
  /// Non-blocking drain of any `ack` responses currently available on the socket,
  /// removing their records from the window. A peer close or io error drops the
  /// connection (mirrors write_bytes's Io handling) but does not throw.
  void drain_acks() noexcept;
  void handle_ack_object(const msgpack::object& obj) noexcept;
  /// Resends every window entry (force_all) or only those past ack_timeout, oldest
  /// first. Stops at the first write failure (connection issue), leaving the remainder
  /// for the next poll()/send(). Never throws.
  void resend(bool force_all) noexcept;
  void enqueue_pending(std::string chunk_id, std::string frame_bytes);
  static std::string generate_chunk_id();

  ForwarderConfig config_;
  int fd_{ -1 };
  msgpack::unpacker ack_unpacker_;
  std::deque<PendingRecord> window_;
  std::size_t window_bytes_{ 0 };
  std::chrono::steady_clock::time_point last_warn_at_{};
  bool warned_once_{ false };
};

}  // namespace dc_bridge

#endif  // DC_BRIDGE__FORWARDER_HPP_
