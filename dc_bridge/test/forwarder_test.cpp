// Frame correctness / tag handling decode the static frame() bytes directly;
// reconnection, backpressure, and ack/window behavior (#266) use a mock shipper ingest
// protocol TCP server.
#include "dc_bridge/forwarder.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <msgpack.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

using namespace dc_bridge;

namespace
{

Record make_record(const std::string& tag, const std::string& json)
{
  return Record{ tag, 1700000000ULL, nlohmann::json::parse(json) };
}

// A mock shipper ingest protocol server: binds :0, hands back the chosen port, and runs
// `handler` on the accepted connection in a background thread.
struct MockServer
{
  int listen_fd{ -1 };
  std::uint16_t port{ 0 };

  MockServer()
  {
    listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sa.sin_port = 0;
    ::bind(listen_fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa));
    ::listen(listen_fd, 4);
    socklen_t len = sizeof(sa);
    ::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&sa), &len);
    port = ntohs(sa.sin_port);
  }
  ~MockServer()
  {
    if (listen_fd >= 0)
      ::close(listen_fd);
  }
};

// Unpacks one msgpack object from `bytes`.
msgpack::object_handle unpack(const std::string& bytes)
{
  return msgpack::unpack(bytes.data(), bytes.size());
}

// Pulls the "chunk" option's value out of a real on-the-wire frame — used by mock
// servers that need to echo back whatever chunk id the Forwarder actually generated
// (its value isn't otherwise observable from outside Forwarder::send()).
std::string extract_chunk_id(const char* bytes, std::size_t len)
{
  msgpack::object_handle oh = msgpack::unpack(bytes, len);
  msgpack::object top = oh.get();
  if (top.via.array.size < 3)
  {
    return {};
  }
  msgpack::object option = top.via.array.ptr[2];
  if (option.type != msgpack::type::MAP)
  {
    return {};
  }
  for (std::uint32_t i = 0; i < option.via.map.size; ++i)
  {
    if (option.via.map.ptr[i].key.as<std::string>() == "chunk")
    {
      return option.via.map.ptr[i].val.as<std::string>();
    }
  }
  return {};
}

// Reads one full frame off `fd` (a single ::recv is enough for these small test
// payloads) and sends back `{"ack": <its chunk id>}`.
void read_frame_and_ack(int fd)
{
  char buf[8192];
  ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
  if (n <= 0)
  {
    return;
  }
  const std::string chunk_id = extract_chunk_id(buf, static_cast<std::size_t>(n));
  msgpack::sbuffer ack_buf;
  msgpack::packer<msgpack::sbuffer> pk(ack_buf);
  pk.pack_map(1);
  pk.pack(std::string("ack"));
  pk.pack(chunk_id);
  ::send(fd, ack_buf.data(), ack_buf.size(), 0);
}

}  // namespace

TEST(Forwarder, FrameIsWellFormedWithCorrectTag)
{
  auto oh = unpack(Forwarder::frame(make_record("dc.measurement.uptime", R"({"uptime_s": 42})"), "chunk-a"));
  msgpack::object top = oh.get();
  ASSERT_EQ(top.type, msgpack::type::ARRAY);
  ASSERT_EQ(top.via.array.size, 3u);  // [tag, entries, option]
  EXPECT_EQ(top.via.array.ptr[0].as<std::string>(), "dc.measurement.uptime");

  msgpack::object entries = top.via.array.ptr[1];
  ASSERT_EQ(entries.via.array.size, 1u);
  msgpack::object entry = entries.via.array.ptr[0];  // [time, record]
  EXPECT_EQ(entry.via.array.ptr[0].as<std::uint64_t>(), 1700000000ULL);

  msgpack::object rec = entry.via.array.ptr[1];
  ASSERT_EQ(rec.type, msgpack::type::MAP);
  bool found = false;
  for (uint32_t i = 0; i < rec.via.map.size; ++i)
  {
    if (rec.via.map.ptr[i].key.as<std::string>() == "uptime_s")
    {
      EXPECT_EQ(rec.via.map.ptr[i].val.as<std::uint64_t>(), 42u);
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST(Forwarder, FrameCarriesTheChunkOption)
{
  auto oh = unpack(Forwarder::frame(make_record("dc.test", R"({"n": 1})"), "my-chunk-id"));
  msgpack::object top = oh.get();
  ASSERT_EQ(top.via.array.size, 3u);
  msgpack::object option = top.via.array.ptr[2];
  ASSERT_EQ(option.type, msgpack::type::MAP);
  ASSERT_EQ(option.via.map.size, 1u);
  EXPECT_EQ(option.via.map.ptr[0].key.as<std::string>(), "chunk");
  EXPECT_EQ(option.via.map.ptr[0].val.as<std::string>(), "my-chunk-id");
}

TEST(Forwarder, NonObjectPayloadsWrappedInMessageField)
{
  auto oh = unpack(Forwarder::frame(make_record("dc.test", R"("plain string data")"), "chunk-b"));
  msgpack::object rec = oh.get().via.array.ptr[1].via.array.ptr[0].via.array.ptr[1];
  ASSERT_EQ(rec.type, msgpack::type::MAP);
  ASSERT_EQ(rec.via.map.size, 1u);
  EXPECT_EQ(rec.via.map.ptr[0].key.as<std::string>(), "message");
  EXPECT_EQ(rec.via.map.ptr[0].val.as<std::string>(), "plain string data");
}

TEST(Forwarder, DifferentRecordsCarryTheirOwnTag)
{
  EXPECT_EQ(unpack(Forwarder::frame(make_record("dc.measurement.cpu", R"({"pct": 1})"), "c1"))
                .get()
                .via.array.ptr[0]
                .as<std::string>(),
            "dc.measurement.cpu");
  EXPECT_EQ(unpack(Forwarder::frame(make_record("dc.measurement.uptime", R"({"uptime_s": 2})"), "c2"))
                .get()
                .via.array.ptr[0]
                .as<std::string>(),
            "dc.measurement.uptime");
}

TEST(Forwarder, ReconnectsAfterPeerDropsConnection)
{
  MockServer server;
  std::thread srv([&server]() {
    // First connection: read one frame, then abortively close (SO_LINGER 0 → RST).
    int c = ::accept(server.listen_fd, nullptr, nullptr);
    char buf[4096];
    ::recv(c, buf, sizeof(buf), 0);
    linger lg{ 1, 0 };
    ::setsockopt(c, SOL_SOCKET, SO_LINGER, &lg, sizeof(lg));
    ::close(c);  // RST
    // Second connection proves the Forwarder reconnected on its own.
    int c2 = ::accept(server.listen_fd, nullptr, nullptr);
    ::recv(c2, buf, sizeof(buf), 0);
    ::close(c2);
  });

  ForwarderConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = server.port;
  Forwarder forwarder(cfg);
  forwarder.send(make_record("dc.test", R"({"n": 1})"));

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  bool reconnected = false;
  for (int i = 0; i < 100; ++i)
  {
    try
    {
      forwarder.send(make_record("dc.test", R"({"n": 2})"));
      reconnected = true;
      break;
    }
    catch (const ForwarderError&)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  srv.join();
  EXPECT_TRUE(reconnected) << "forwarder should reconnect after the peer resets the connection";
}

TEST(Forwarder, ReturnsBackpressureWhenPeerStalls)
{
  MockServer server;
  std::thread srv([&server]() {
    int c = ::accept(server.listen_fd, nullptr, nullptr);
    int small = 1024;
    ::setsockopt(c, SOL_SOCKET, SO_RCVBUF, &small, sizeof(small));
    std::this_thread::sleep_for(std::chrono::seconds(2));  // never read
    ::close(c);
  });

  ForwarderConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = server.port;
  cfg.write_timeout = std::chrono::milliseconds(50);
  Forwarder forwarder(cfg);

  nlohmann::json big;
  big["blob"] = std::string(1000000, 'x');
  Record big_record{ "dc.test", 1700000000ULL, big };

  bool saw_backpressure = false;
  for (int i = 0; i < 20; ++i)
  {
    try
    {
      forwarder.send(big_record);
    }
    catch (const ForwarderError& e)
    {
      if (e.kind() == ForwarderErrorKind::Backpressure)
      {
        saw_backpressure = true;
        break;
      }
    }
  }
  srv.join();
  EXPECT_TRUE(saw_backpressure) << "expected a stalled peer to eventually trigger backpressure";
}

TEST(Forwarder, AckedRecordLeavesTheUnackedWindow)
{
  MockServer server;
  std::thread srv([&]() {
    int c = ::accept(server.listen_fd, nullptr, nullptr);
    if (c < 0)
    {
      return;
    }
    read_frame_and_ack(c);  // exactly one Record is sent in this test
    ::close(c);
  });

  ForwarderConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = server.port;
  Forwarder forwarder(cfg);

  forwarder.send(make_record("dc.test", R"({"n": 1})"));
  EXPECT_EQ(forwarder.unacked_count(), 1u);

  bool drained = false;
  for (int i = 0; i < 100; ++i)
  {
    forwarder.poll();
    if (forwarder.unacked_count() == 0)
    {
      drained = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_TRUE(drained) << "an acked record should leave the unacked window";
  EXPECT_EQ(forwarder.unacked_bytes(), 0u);

  srv.join();
}

TEST(Forwarder, UnackedRecordResendsAfterReconnect)
{
  MockServer server;
  std::vector<std::string> chunk_ids_seen;
  std::mutex seen_mutex;

  std::thread srv([&]() {
    // First connection: read the frame, note its chunk id, never ack it, then abort the
    // connection (RST) without responding — simulating Vector dying mid-flight.
    int c1 = ::accept(server.listen_fd, nullptr, nullptr);
    char buf[4096];
    ssize_t n = ::recv(c1, buf, sizeof(buf), 0);
    {
      std::lock_guard<std::mutex> lock(seen_mutex);
      chunk_ids_seen.push_back(extract_chunk_id(buf, static_cast<std::size_t>(n)));
    }
    linger lg{ 1, 0 };
    ::setsockopt(c1, SOL_SOCKET, SO_LINGER, &lg, sizeof(lg));
    ::close(c1);

    // Second connection: the Forwarder should redeliver the same unacked record before
    // (or as part of) sending anything new. Ack it this time so the test can also see
    // the window converge.
    int c2 = ::accept(server.listen_fd, nullptr, nullptr);
    ssize_t n2 = ::recv(c2, buf, sizeof(buf), 0);
    {
      std::lock_guard<std::mutex> lock(seen_mutex);
      chunk_ids_seen.push_back(extract_chunk_id(buf, static_cast<std::size_t>(n2)));
    }
    const std::string chunk_id = extract_chunk_id(buf, static_cast<std::size_t>(n2));
    msgpack::sbuffer ack_buf;
    msgpack::packer<msgpack::sbuffer> pk(ack_buf);
    pk.pack_map(1);
    pk.pack(std::string("ack"));
    pk.pack(chunk_id);
    ::send(c2, ack_buf.data(), ack_buf.size(), 0);
    ::close(c2);
  });

  ForwarderConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = server.port;
  Forwarder forwarder(cfg);
  forwarder.send(make_record("dc.test", R"({"n": 1})"));

  // Give the mock server time to RST the first connection, then poll until the
  // Forwarder has reconnected and redelivered.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  bool redelivered = false;
  for (int i = 0; i < 100; ++i)
  {
    forwarder.poll();
    std::lock_guard<std::mutex> lock(seen_mutex);
    if (chunk_ids_seen.size() >= 2)
    {
      redelivered = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  srv.join();

  ASSERT_TRUE(redelivered) << "the unacked record should have been resent on the new connection";
  std::lock_guard<std::mutex> lock(seen_mutex);
  ASSERT_EQ(chunk_ids_seen.size(), 2u);
  EXPECT_EQ(chunk_ids_seen[0], chunk_ids_seen[1]) << "the resend must carry the SAME chunk id as the original send";
}

TEST(Forwarder, WindowBoundDropsOldestRecordsWithWarning)
{
  MockServer server;
  std::atomic<bool> stop{ false };
  std::thread srv([&]() {
    // Accept the connection but never read/ack anything, so every send() stays
    // unacked and the window is forced to enforce its bound.
    int c = ::accept(server.listen_fd, nullptr, nullptr);
    while (!stop.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (c >= 0)
    {
      ::close(c);
    }
  });

  ForwarderConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = server.port;
  cfg.max_unacked_records = 3;
  cfg.max_unacked_bytes = 1024 * 1024;  // large enough that the record-count bound trips first
  cfg.warn_interval = std::chrono::milliseconds(0);
  std::atomic<int> warnings{ 0 };
  std::string last_warning;
  cfg.on_warning = [&](const std::string& msg) {
    ++warnings;
    last_warning = msg;
  };
  Forwarder forwarder(cfg);

  for (int i = 0; i < 5; ++i)
  {
    forwarder.send(make_record("dc.test", R"({"n": 1})"));
  }

  EXPECT_EQ(forwarder.unacked_count(), 3u) << "the window must be capped at max_unacked_records";
  EXPECT_GT(warnings.load(), 0) << "dropping records past the bound must warn";
  EXPECT_FALSE(last_warning.empty());

  stop.store(true);
  srv.join();
}
