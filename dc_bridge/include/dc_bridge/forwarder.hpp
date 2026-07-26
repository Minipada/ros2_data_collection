// Forwarder: send(record) hides Fluent Forward msgpack framing, socket lifecycle,
// reconnection, and backpressure behind one call. The wire protocol is the interface;
// nothing else about Vector or the transport leaks past this class.
#ifndef DC_BRIDGE__FORWARDER_HPP_
#define DC_BRIDGE__FORWARDER_HPP_

#include <chrono>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>

namespace dc_bridge
{

/// A DC Record ready to be forwarded: a Fluent Forward tag, a Unix timestamp (seconds
/// since the epoch, matching Fluent Forward's integer time), and the Record's JSON
/// payload (StringStamped.data, already parsed).
struct Record
{
  std::string tag;
  std::uint64_t timestamp_secs;
  nlohmann::json payload;
};

struct ForwarderConfig
{
  std::string host;
  std::uint16_t port;
  std::chrono::milliseconds connect_timeout{ 2000 };
  std::chrono::milliseconds write_timeout{ 200 };
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

/// Sends Records to a Fluent Forward peer (Vector's `fluent` source) over TCP.
///
/// Reconnects lazily: a dead connection is dropped on write failure and re-established
/// on the next send(). A write that blocks past write_timeout is reported as
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

  /// Sends one Record. Throws ForwarderError on failure (see ForwarderErrorKind).
  void send(const Record& record);

  /// The Fluent Forward "Forward Mode" frame `[tag, [[time, record]]]` for `record`,
  /// as msgpack bytes. Exposed for the wire-format tests.
  static std::string frame(const Record& record);

private:
  void ensure_connected();
  void close_connection() noexcept;

  ForwarderConfig config_;
  int fd_{ -1 };
};

}  // namespace dc_bridge

#endif  // DC_BRIDGE__FORWARDER_HPP_
