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
#include <msgpack.hpp>

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

// The Fluent Forward wire record must be a map. A Record's JSON payload usually is one;
// anything else (bare string/number/array) is wrapped in {"message": ...} rather than
// dropped.
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
}

std::string Forwarder::frame(const Record& record)
{
  msgpack::sbuffer buf;
  msgpack::packer<msgpack::sbuffer> pk(buf);
  // [tag, [[time, record]]]
  pk.pack_array(2);
  pk.pack(record.tag);
  pk.pack_array(1);  // one entry
  pk.pack_array(2);  // [time, record]
  pk.pack(record.timestamp_secs);
  pack_record_map(pk, record.payload);
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
    throw ForwarderError(ForwarderErrorKind::Connect, "failed to connect to Fluent Forward peer at " + addr_str +
                                                          ": socket(): " + std::strerror(errno));
  }

  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_port = htons(config_.port);
  if (::inet_pton(AF_INET, config_.host.c_str(), &sa.sin_addr) != 1)
  {
    ::close(fd);
    throw ForwarderError(ForwarderErrorKind::Connect,
                         "failed to connect to Fluent Forward peer at " + addr_str + ": invalid host address");
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
                           "failed to connect to Fluent Forward peer at " + addr_str + ": connect timed out");
    }
    int soerr = 0;
    socklen_t len = sizeof(soerr);
    ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len);
    if (soerr != 0)
    {
      ::close(fd);
      throw ForwarderError(ForwarderErrorKind::Connect,
                           "failed to connect to Fluent Forward peer at " + addr_str + ": " + std::strerror(soerr));
    }
  }
  else if (rc < 0)
  {
    ::close(fd);
    throw ForwarderError(ForwarderErrorKind::Connect,
                         "failed to connect to Fluent Forward peer at " + addr_str + ": " + std::strerror(errno));
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
}

void Forwarder::send(const Record& record)
{
  ensure_connected();
  const std::string frame_bytes = frame(record);

  std::size_t written = 0;
  while (written < frame_bytes.size())
  {
    // MSG_NOSIGNAL: a dead peer must surface as EPIPE (Io), never a process-killing
    // SIGPIPE.
    ssize_t n = ::send(fd_, frame_bytes.data() + written, frame_bytes.size() - written, MSG_NOSIGNAL);
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

}  // namespace dc_bridge
