// Port of dc_bridge_core forwarder_tests: frame correctness / tag handling decode the
// static frame() bytes directly; reconnection and backpressure use a mock Fluent Forward
// TCP server, matching the Rust acceptance tests.
#include "dc_bridge/forwarder.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <msgpack.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

using namespace dc_bridge;

namespace
{

Record make_record(const std::string& tag, const std::string& json)
{
  return Record{ tag, 1700000000ULL, nlohmann::json::parse(json) };
}

// A mock Fluent Forward server: binds :0, hands back the chosen port, and runs `handler`
// on the accepted connection in a background thread.
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

}  // namespace

TEST(Forwarder, FrameIsWellFormedWithCorrectTag)
{
  auto oh = unpack(Forwarder::frame(make_record("dc.measurement.uptime", R"({"uptime_s": 42})")));
  msgpack::object top = oh.get();
  ASSERT_EQ(top.type, msgpack::type::ARRAY);
  ASSERT_EQ(top.via.array.size, 2u);  // [tag, entries]
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

TEST(Forwarder, NonObjectPayloadsWrappedInMessageField)
{
  auto oh = unpack(Forwarder::frame(make_record("dc.test", R"("plain string data")")));
  msgpack::object rec = oh.get().via.array.ptr[1].via.array.ptr[0].via.array.ptr[1];
  ASSERT_EQ(rec.type, msgpack::type::MAP);
  ASSERT_EQ(rec.via.map.size, 1u);
  EXPECT_EQ(rec.via.map.ptr[0].key.as<std::string>(), "message");
  EXPECT_EQ(rec.via.map.ptr[0].val.as<std::string>(), "plain string data");
}

TEST(Forwarder, DifferentRecordsCarryTheirOwnTag)
{
  EXPECT_EQ(unpack(Forwarder::frame(make_record("dc.measurement.cpu", R"({"pct": 1})")))
                .get()
                .via.array.ptr[0]
                .as<std::string>(),
            "dc.measurement.cpu");
  EXPECT_EQ(unpack(Forwarder::frame(make_record("dc.measurement.uptime", R"({"uptime_s": 2})")))
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
