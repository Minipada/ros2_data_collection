#include "dc_bridge/forwarder.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>
#include <fstream>
#include <msgpack.hpp>
#include <random>

namespace dc_bridge
{

namespace
{

// Packs a nlohmann::json value onto a msgpack packer: numbers keep
// integer/unsigned/float distinctions, everything else maps 1:1. (msgpack-cxx has no
// built-in nlohmann::json adaptor, so we walk the tree.)
template <typename Packer>
void pack_json(Packer& pk, const nlohmann::json& value)
{
  switch (value.type())
  {
    case nlohmann::json::value_t::null:
      pk.pack_nil();
      break;
    case nlohmann::json::value_t::boolean:
      pk.pack(value.get<bool>());
      break;
    case nlohmann::json::value_t::number_integer:
      pk.pack(value.get<std::int64_t>());
      break;
    case nlohmann::json::value_t::number_unsigned:
      pk.pack(value.get<std::uint64_t>());
      break;
    case nlohmann::json::value_t::number_float:
      pk.pack(value.get<double>());
      break;
    case nlohmann::json::value_t::string:
      pk.pack(value.get<std::string>());
      break;
    case nlohmann::json::value_t::array:
      pk.pack_array(static_cast<std::uint32_t>(value.size()));
      for (const auto& elem : value)
      {
        pack_json(pk, elem);
      }
      break;
    case nlohmann::json::value_t::object:
      pk.pack_map(static_cast<std::uint32_t>(value.size()));
      for (auto it = value.begin(); it != value.end(); ++it)
      {
        pk.pack(it.key());
        pack_json(pk, it.value());
      }
      break;
    default:
      pk.pack_nil();
      break;
  }
}

// The shipper ingest protocol's wire record must be a map. A Record's JSON payload
// usually is one; anything else (bare string/number/array) is wrapped in
// {"message": ...} rather than dropped.
template <typename Packer>
void pack_record_map(Packer& pk, const nlohmann::json& payload)
{
  if (payload.is_object())
  {
    pack_json(pk, payload);
  }
  else
  {
    pk.pack_map(1);
    pk.pack(std::string("message"));
    pack_json(pk, payload);
  }
}

}  // namespace

Forwarder::Forwarder(ForwarderConfig config) : config_(std::move(config))
{
}

Forwarder::~Forwarder()
{
  close_connection();
}

void Forwarder::close_connection() noexcept
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
  ack_unpacker_.reset();
}

std::string Forwarder::generate_chunk_id()
{
  // 16 random bytes, base64-encoded — the protocol only requires a unique-enough opaque
  // id that the server echoes back verbatim in its ack response.
  static thread_local std::mt19937_64 rng{ std::random_device{}() };
  std::uint8_t bytes[16];
  for (std::size_t i = 0; i < sizeof(bytes); i += sizeof(std::uint64_t))
  {
    const std::uint64_t v = rng();
    std::memcpy(bytes + i, &v, std::min(sizeof(v), sizeof(bytes) - i));
  }

  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(24);
  for (std::size_t i = 0; i < sizeof(bytes); i += 3)
  {
    const std::uint32_t n0 = bytes[i];
    const std::uint32_t n1 = (i + 1 < sizeof(bytes)) ? bytes[i + 1] : 0;
    const std::uint32_t n2 = (i + 2 < sizeof(bytes)) ? bytes[i + 2] : 0;
    const std::uint32_t triple = (n0 << 16) | (n1 << 8) | n2;
    out.push_back(alphabet[(triple >> 18) & 0x3F]);
    out.push_back(alphabet[(triple >> 12) & 0x3F]);
    if (i + 1 < sizeof(bytes))
    {
      out.push_back(alphabet[(triple >> 6) & 0x3F]);
    }
    if (i + 2 < sizeof(bytes))
    {
      out.push_back(alphabet[triple & 0x3F]);
    }
  }
  return out;
}

std::string Forwarder::frame(const Record& record, const std::string& chunk_id)
{
  msgpack::sbuffer buf;
  msgpack::packer<msgpack::sbuffer> pk(buf);
  // [tag, [[time, record]], {"chunk": chunk_id}] — the "option" third element is what
  // asks the peer to ack this chunk (#266).
  pk.pack_array(3);
  pk.pack(record.tag);
  pk.pack_array(1);  // one entry
  pk.pack_array(2);  // [time, record]
  pk.pack(record.timestamp_secs);
  pack_record_map(pk, record.payload);
  pk.pack_map(1);
  pk.pack(std::string("chunk"));
  pk.pack(chunk_id);
  return std::string(buf.data(), buf.size());
}

void Forwarder::ensure_connected()
{
  if (fd_ >= 0)
  {
    return;
  }

  const std::string addr_str = config_.host + ":" + std::to_string(config_.port);

  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
  {
    throw ForwarderError(ForwarderErrorKind::Connect, "failed to connect to the shipper ingest peer at " + addr_str +
                                                          ": socket(): " + std::strerror(errno));
  }

  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_port = htons(config_.port);
  if (::inet_pton(AF_INET, config_.host.c_str(), &sa.sin_addr) != 1)
  {
    ::close(fd);
    throw ForwarderError(ForwarderErrorKind::Connect,
                         "failed to connect to the shipper ingest peer at " + addr_str + ": invalid host address");
  }

  // Non-blocking connect + poll for connect_timeout, so a black-holed peer doesn't
  // block startup for the OS default of ~minutes.
  int flags = ::fcntl(fd, F_GETFL, 0);
  ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  int rc = ::connect(fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa));
  if (rc < 0 && errno == EINPROGRESS)
  {
    pollfd pfd{ fd, POLLOUT, 0 };
    int pr = ::poll(&pfd, 1, static_cast<int>(config_.connect_timeout.count()));
    if (pr <= 0)
    {
      ::close(fd);
      throw ForwarderError(ForwarderErrorKind::Connect,
                           "failed to connect to the shipper ingest peer at " + addr_str + ": connect timed out");
    }
    int soerr = 0;
    socklen_t len = sizeof(soerr);
    ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len);
    if (soerr != 0)
    {
      ::close(fd);
      throw ForwarderError(ForwarderErrorKind::Connect,
                           "failed to connect to the shipper ingest peer at " + addr_str + ": " + std::strerror(soerr));
    }
  }
  else if (rc < 0)
  {
    ::close(fd);
    throw ForwarderError(ForwarderErrorKind::Connect,
                         "failed to connect to the shipper ingest peer at " + addr_str + ": " + std::strerror(errno));
  }
  ::fcntl(fd, F_SETFL, flags);  // back to blocking; write_timeout is enforced below

  // Backpressure detection: a write that blocks past write_timeout returns EAGAIN.
  timeval tv{};
  tv.tv_sec = config_.write_timeout.count() / 1000;
  tv.tv_usec = (config_.write_timeout.count() % 1000) * 1000;
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  int one = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

  fd_ = fd;
  ack_unpacker_.reset();  // a fresh connection is a fresh msgpack byte stream
}

void Forwarder::write_bytes(const std::string& bytes)
{
  std::size_t written = 0;
  while (written < bytes.size())
  {
    // MSG_NOSIGNAL: a dead peer must surface as EPIPE (Io), never a process-killing
    // SIGPIPE.
    ssize_t n = ::send(fd_, bytes.data() + written, bytes.size() - written, MSG_NOSIGNAL);
    if (n > 0)
    {
      written += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      // Peer stalled: keep the connection (it may just be slow to drain), report
      // backpressure.
      throw ForwarderError(ForwarderErrorKind::Backpressure,
                           "peer stalled: write did not complete within the configured timeout");
    }
    // Any other error drops the connection so the next send() reconnects.
    const std::string msg = std::string("io error while forwarding record: ") + std::strerror(errno);
    close_connection();
    throw ForwarderError(ForwarderErrorKind::Io, msg);
  }
}

void Forwarder::handle_ack_object(const msgpack::object& obj) noexcept
{
  if (obj.type != msgpack::type::MAP)
  {
    return;
  }
  for (std::uint32_t i = 0; i < obj.via.map.size; ++i)
  {
    const msgpack::object_kv& kv = obj.via.map.ptr[i];
    if (kv.key.type != msgpack::type::STR)
    {
      continue;
    }
    try
    {
      if (kv.key.as<std::string>() != "ack" || kv.val.type != msgpack::type::STR)
      {
        continue;
      }
      const std::string acked = kv.val.as<std::string>();
      for (auto it = window_.begin(); it != window_.end(); ++it)
      {
        if (it->chunk_id == acked)
        {
          window_bytes_ -= it->frame_bytes.size();
          window_.erase(it);
          break;
        }
      }
    }
    catch (const std::exception&)
    {
      // Malformed ack payload from the peer; nothing in the window matched, ignore.
    }
  }
}

void Forwarder::drain_acks() noexcept
{
  if (fd_ < 0)
  {
    return;
  }
  char buf[4096];
  for (;;)
  {
    ssize_t n = ::recv(fd_, buf, sizeof(buf), MSG_DONTWAIT);
    if (n > 0)
    {
      try
      {
        ack_unpacker_.reserve_buffer(static_cast<std::size_t>(n));
        std::memcpy(ack_unpacker_.buffer(), buf, static_cast<std::size_t>(n));
        ack_unpacker_.buffer_consumed(static_cast<std::size_t>(n));
        msgpack::object_handle oh;
        while (ack_unpacker_.next(oh))
        {
          handle_ack_object(oh.get());
        }
      }
      catch (const std::exception&)
      {
        // Corrupt ack stream; drop the connection so the next send()/poll() starts
        // clean rather than getting stuck on a desynced unpacker.
        close_connection();
        return;
      }
      continue;
    }
    if (n == 0)
    {
      close_connection();  // peer closed
      return;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
      return;  // no more data available right now
    }
    close_connection();  // real socket error
    return;
  }
}

void Forwarder::resend(bool force_all) noexcept
{
  const auto now = std::chrono::steady_clock::now();
  for (auto& entry : window_)
  {
    if (!force_all && (now - entry.sent_at) < config_.ack_timeout)
    {
      continue;
    }
    try
    {
      write_bytes(entry.frame_bytes);
      entry.sent_at = now;
    }
    catch (const ForwarderError&)
    {
      return;  // connection dropped or stalled; retry the rest on the next opportunity
    }
  }
}

void Forwarder::enqueue_pending(std::string chunk_id, std::string frame_bytes)
{
  window_bytes_ += frame_bytes.size();
  window_.push_back(PendingRecord{ std::move(chunk_id), std::move(frame_bytes), std::chrono::steady_clock::now() });

  std::size_t dropped = 0;
  while (!window_.empty() && (window_.size() > config_.max_unacked_records || window_bytes_ > config_.max_unacked_bytes))
  {
    window_bytes_ -= window_.front().frame_bytes.size();
    window_.pop_front();
    ++dropped;
  }

  if (dropped == 0)
  {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (warned_once_ && (now - last_warn_at_) < config_.warn_interval)
  {
    return;  // rate-limited
  }
  warned_once_ = true;
  last_warn_at_ = now;
  if (config_.on_warning)
  {
    config_.on_warning("unacked record window exceeded its bound (" + std::to_string(config_.max_unacked_records) +
                       " records / " + std::to_string(config_.max_unacked_bytes) +
                       " bytes); dropped the oldest unacked record(s) to make room");
  }
}

void Forwarder::send(const Record& record)
{
  const bool was_disconnected = (fd_ < 0);
  ensure_connected();
  if (was_disconnected)
  {
    resend(/*force_all=*/true);
  }
  drain_acks();
  resend(/*force_all=*/false);

  // drain_acks()/resend() above may have dropped the connection on a stale peer; make
  // sure a fresh one is up (and, if it's genuinely new, redeliver the window) before
  // sending the record this call is actually for.
  const bool reconnected_for_send = (fd_ < 0);
  ensure_connected();
  if (reconnected_for_send)
  {
    resend(/*force_all=*/true);
  }

  const std::string chunk_id = generate_chunk_id();
  const std::string frame_bytes = frame(record, chunk_id);
  enqueue_pending(chunk_id, frame_bytes);
  write_bytes(frame_bytes);
}

void Forwarder::poll() noexcept
{
  if (fd_ < 0 && window_.empty())
  {
    return;  // nothing connected and nothing to flush
  }
  try
  {
    const bool was_disconnected = (fd_ < 0);
    ensure_connected();
    if (was_disconnected)
    {
      resend(/*force_all=*/true);
    }
  }
  catch (const ForwarderError&)
  {
    return;  // still unreachable; try again next poll()/send()
  }
  drain_acks();
  resend(/*force_all=*/false);
}

}  // namespace dc_bridge
