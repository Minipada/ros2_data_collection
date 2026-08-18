// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

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
#include <functional>
#include <msgpack.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <thread>
#include <vector>

using namespace dc_bridge;

namespace
{

Record make_record(const std::string& tag, const std::string& json, std::uint32_t nanos = 0)
{
  return Record{ tag, 1700000000ULL, nanos, nlohmann::json::parse(json) };
}

// The EventTime extension the frame carries the timestamp in: msgpack ext type 0x00,
// 8-byte body of big-endian seconds then big-endian nanoseconds (#308). Built here
// independently of the packing code so the test pins the wire bytes rather than
// re-deriving them the same way the implementation does.
std::string expected_event_time_bytes(std::uint32_t secs, std::uint32_t nanos)
{
  std::string out;
  out.push_back(static_cast<char>(0xd7));  // fixext8
  out.push_back(static_cast<char>(0x00));  // ext type 0 = EventTime
  for (int i = 0; i < 4; ++i)
  {
    out.push_back(static_cast<char>((secs >> (8 * (3 - i))) & 0xFF));
  }
  for (int i = 0; i < 4; ++i)
  {
    out.push_back(static_cast<char>((nanos >> (8 * (3 - i))) & 0xFF));
  }
  return out;
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

// Polls `condition` until it returns true or `budget` of real wall-clock time elapses,
// sleeping 20ms between checks. A wall-clock deadline rather than a fixed iteration
// count: a fixed `for (i < N) { ...; sleep(20ms); }` loop implicitly assumes each
// iteration costs ~20ms, but scheduler contention or a slow (e.g. coverage-instrumented)
// build inflates the real cost per iteration, silently shrinking the test's actual
// patience without touching the visible "N iterations" budget — the CI flake this
// replaced (Forwarder.UnackedRecordResendsAfterReconnect intermittently failing under a
// loaded/coverage build) was exactly that. `condition` may have side effects (e.g.
// calling forwarder.poll()) — it's invoked once more after the deadline so a
// just-in-time success right at the boundary still counts.
bool poll_until(std::chrono::milliseconds budget, const std::function<bool()>& condition)
{
  const auto deadline = std::chrono::steady_clock::now() + budget;
  do
  {
    if (condition())
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  } while (std::chrono::steady_clock::now() < deadline);
  return condition();
}

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

// --- Acknowledgement-policy mock ingest peer (#366) ---------------------------------
//
// Real robots ack over a link with tens of milliseconds of RTT, packet loss, and a
// bufferbloated/slow-draining receive path — none of which loopback (the rest of this
// file) ever exercises. A policy turns the three questions that raises about the
// Forwarder — unacked window depth, backpressure classification, resend-under-delay —
// into deterministic unit tests: no container, no network, just a peer that
// acknowledges on a schedule instead of immediately or never. Every existing test above
// is this policy's degenerate case (delay=0, never_ack_probability=0).
struct AckPolicy
{
  // Delay applied before acknowledging a received chunk — models acknowledgement RTT.
  std::chrono::milliseconds ack_delay{ 0 };
  // Fraction of received chunks that are never acknowledged — models a lost ack.
  double never_ack_probability{ 0.0 };
  // Bytes read per recv() before sleeping drain_pause; 0 = read as fast as available.
  // Models a peer that is draining its socket slowly rather than one that stopped
  // reading altogether (drain_pause with drain_chunk_bytes == 0 would never actually
  // throttle, since one recv() would drain everything available).
  std::size_t drain_chunk_bytes{ 0 };
  std::chrono::milliseconds drain_pause{ 0 };
};

// Runs `policy` against one already-accepted connection until the peer closes/errors or
// `stop` is set. Every chunk id the frame parser completes is appended to `received` (in
// wire order, so a resend that duplicates a chunk id shows up twice); every chunk id
// actually acknowledged is appended to `acked`. Uses a real msgpack::unpacker (exactly
// like Forwarder's own drain_acks()) so a small drain_chunk_bytes that splits one frame
// across several recv() calls is still parsed correctly, rather than requiring every
// frame to land in a single read.
void run_ack_policy_peer(int conn_fd, const AckPolicy& policy, std::vector<std::string>& received,
                         std::vector<std::string>& acked, std::mutex& mutex, std::atomic<bool>& stop)
{
  std::mt19937 rng{ std::random_device{}() };
  std::uniform_real_distribution<double> unit{ 0.0, 1.0 };
  msgpack::unpacker unpacker;

  // A short receive timeout, not a blocking recv(): once the client under test goes idle
  // (e.g. everything already acked), a blocking recv() would never return and `stop`
  // would never be re-checked, hanging the test's own srv.join() forever.
  timeval rcv_timeout{ 0, 100000 };  // 100ms
  ::setsockopt(conn_fd, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));

  while (!stop.load())
  {
    const std::size_t to_read = policy.drain_chunk_bytes == 0 ? std::size_t{ 8192 } : policy.drain_chunk_bytes;
    unpacker.reserve_buffer(to_read);
    ssize_t n = ::recv(conn_fd, unpacker.buffer(), to_read, 0);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      continue;  // no data within the receive timeout; re-check `stop` and try again
    }
    if (n <= 0)
    {
      return;  // peer closed or errored
    }
    unpacker.buffer_consumed(static_cast<std::size_t>(n));
    if (policy.drain_pause.count() > 0)
    {
      std::this_thread::sleep_for(policy.drain_pause);
    }

    msgpack::object_handle oh;
    while (unpacker.next(oh))
    {
      msgpack::object top = oh.get();
      if (top.type != msgpack::type::ARRAY || top.via.array.size < 3)
      {
        continue;
      }
      msgpack::object option = top.via.array.ptr[2];
      std::string chunk_id;
      if (option.type == msgpack::type::MAP)
      {
        for (std::uint32_t i = 0; i < option.via.map.size; ++i)
        {
          if (option.via.map.ptr[i].key.as<std::string>() == "chunk")
          {
            chunk_id = option.via.map.ptr[i].val.as<std::string>();
          }
        }
      }
      if (chunk_id.empty())
      {
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(mutex);
        received.push_back(chunk_id);
      }
      if (unit(rng) < policy.never_ack_probability)
      {
        continue;
      }
      if (policy.ack_delay.count() > 0)
      {
        std::this_thread::sleep_for(policy.ack_delay);
      }
      msgpack::sbuffer ack_buf;
      msgpack::packer<msgpack::sbuffer> pk(ack_buf);
      pk.pack_map(1);
      pk.pack(std::string("ack"));
      pk.pack(chunk_id);
      if (::send(conn_fd, ack_buf.data(), ack_buf.size(), MSG_NOSIGNAL) < 0)
      {
        return;
      }
      {
        std::lock_guard<std::mutex> lock(mutex);
        acked.push_back(chunk_id);
      }
    }
  }
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
  // The time element is the EventTime extension, not a bare integer of seconds (#308).
  EXPECT_EQ(entry.via.array.ptr[0].type, msgpack::type::EXT);

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

// #308: sub-second resolution must survive the wire. Before this, the frame carried a
// plain integer of whole seconds, so a Measurement polling faster than 1 Hz produced
// Records that all claimed the same instant.
TEST(Forwarder, FrameCarriesSubSecondPrecisionAsEventTime)
{
  const std::uint32_t nanos = 123456789u;
  const std::string frame = Forwarder::frame(make_record("dc.test", R"({"n": 1})", nanos), "chunk-ns");
  EXPECT_NE(frame.find(expected_event_time_bytes(1700000000u, nanos)), std::string::npos)
      << "frame does not carry the EventTime extension for 1700000000.123456789";
}

// Two Records one nanosecond apart must not collapse onto the same wire timestamp —
// the property that makes a timestamp usable as part of an identity.
TEST(Forwarder, FramesWithinTheSameSecondAreDistinguishable)
{
  const std::string a = Forwarder::frame(make_record("dc.test", R"({"n": 1})", 1u), "c1");
  const std::string b = Forwarder::frame(make_record("dc.test", R"({"n": 1})", 2u), "c1");
  EXPECT_NE(a, b);
  EXPECT_NE(a.find(expected_event_time_bytes(1700000000u, 1u)), std::string::npos);
  EXPECT_NE(b.find(expected_event_time_bytes(1700000000u, 2u)), std::string::npos);
}

// A whole-second Record still encodes as EventTime, with a zero nanosecond half, rather
// than falling back to the integer form.
TEST(Forwarder, WholeSecondTimestampStillUsesEventTime)
{
  const std::string frame = Forwarder::frame(make_record("dc.test", R"({"n": 1})", 0u), "c0");
  EXPECT_NE(frame.find(expected_event_time_bytes(1700000000u, 0u)), std::string::npos);
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

  bool reconnected = poll_until(std::chrono::seconds(10), [&]() {
    try
    {
      forwarder.send(make_record("dc.test", R"({"n": 2})"));
      return true;
    }
    catch (const ForwarderError&)
    {
      return false;
    }
  });
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
  Record big_record{ "dc.test", 1700000000ULL, 0u, big };

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

  bool drained = poll_until(std::chrono::seconds(10), [&]() {
    forwarder.poll();
    return forwarder.unacked_count() == 0;
  });
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
  bool redelivered = poll_until(std::chrono::seconds(10), [&]() {
    forwarder.poll();
    std::lock_guard<std::mutex> lock(seen_mutex);
    return chunk_ids_seen.size() >= 2;
  });
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

// #366: the drop-with-warning mechanism above is already covered by a peer that never
// acks at all. What's untested is the *shipped default* bound (10000 records) at an
// acknowledgement delay representative of a real link — this is that measurement.
TEST(Forwarder, WindowDepthStaysWellBelowShippedDefaultBoundAtRealisticAckDelay)
{
  MockServer server;
  std::atomic<bool> stop{ false };
  std::vector<std::string> received, acked;
  std::mutex mutex;
  AckPolicy policy;
  policy.ack_delay = std::chrono::milliseconds(50);  // representative WiFi/uplink ack RTT

  std::thread srv([&]() {
    int c = ::accept(server.listen_fd, nullptr, nullptr);
    if (c >= 0)
    {
      run_ack_policy_peer(c, policy, received, acked, mutex, stop);
      ::close(c);
    }
  });

  ForwarderConfig cfg;  // shipped defaults: max_unacked_records = 10000
  cfg.host = "127.0.0.1";
  cfg.port = server.port;
  std::atomic<int> warnings{ 0 };
  cfg.on_warning = [&](const std::string&) { ++warnings; };
  Forwarder forwarder(cfg);

  const int kRecords = 20;
  std::size_t peak_depth = 0;
  for (int i = 0; i < kRecords; ++i)
  {
    forwarder.send(make_record("dc.test", R"({"n": 1})"));
    peak_depth = std::max(peak_depth, forwarder.unacked_count());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // ~100 Hz, a realistic Measurement rate
  }

  poll_until(std::chrono::seconds(5), [&]() {
    forwarder.poll();
    return forwarder.unacked_count() == 0;
  });

  stop.store(true);
  srv.join();

  // In-flight volume ~= rate * ack delay: at 100 Hz / 50ms that's ~5 records, nowhere
  // near the shipped 10000-record bound.
  EXPECT_LE(peak_depth, 20u) << "unacked window depth at a realistic rate/delay should stay small";
  EXPECT_EQ(warnings.load(), 0) << "the shipped default bound must not fire at a realistic bandwidth-delay product";
}

// #366: unlike WindowBoundDropsOldestRecordsWithWarning (a peer that never acks at all),
// this peer acks every chunk — just too slowly to keep the window inside a small bound
// once the send rate outruns the ack turnaround. Demonstrates the drop valve fires from
// delay alone, not only from a dead/absent peer, and that the window still converges
// once the burst ends.
TEST(Forwarder, WindowBoundIsExceededAtAWorseRateDelayCombination)
{
  MockServer server;
  std::atomic<bool> stop{ false };
  std::vector<std::string> received, acked;
  std::mutex mutex;
  AckPolicy policy;
  policy.ack_delay = std::chrono::milliseconds(300);  // long relative to the burst below

  std::thread srv([&]() {
    int c = ::accept(server.listen_fd, nullptr, nullptr);
    if (c >= 0)
    {
      run_ack_policy_peer(c, policy, received, acked, mutex, stop);
      ::close(c);
    }
  });

  ForwarderConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = server.port;
  cfg.max_unacked_records = 5;
  cfg.max_unacked_bytes = 1024 * 1024;
  cfg.warn_interval = std::chrono::milliseconds(0);
  std::atomic<int> warnings{ 0 };
  cfg.on_warning = [&](const std::string&) { ++warnings; };
  Forwarder forwarder(cfg);

  // A burst well faster than the 300ms ack delay: every record above the bound of 5 is
  // sent before the first ack can possibly have arrived.
  for (int i = 0; i < 10; ++i)
  {
    forwarder.send(make_record("dc.test", R"({"n": 1})"));
  }

  EXPECT_EQ(forwarder.unacked_count(), 5u) << "the bound must cap the window even though the peer is actively acking";
  EXPECT_GT(warnings.load(), 0) << "exceeding the bound via delay alone must still warn";

  bool converged = poll_until(std::chrono::seconds(5), [&]() {
    forwarder.poll();
    return forwarder.unacked_count() == 0;
  });
  stop.store(true);
  srv.join();
  EXPECT_TRUE(converged) << "once the burst ends, the window should still drain to zero — the bound doesn't wedge it";
}

// #366: distinguishes a peer that is slow to drain its socket (retryable backpressure,
// connection kept) from one that has gone away (Io, connection dropped) — the two error
// kinds a real congested/bufferbloated link and a dead peer must not be confused as.
TEST(Forwarder, SlowDrainingPeerIsBackpressureWithConnectionRetained)
{
  MockServer server;
  std::atomic<bool> stop{ false };
  std::vector<std::string> received, acked;
  std::mutex mutex;
  AckPolicy policy;
  policy.drain_chunk_bytes = 16;                       // read a few bytes at a time...
  policy.drain_pause = std::chrono::milliseconds(50);  // ...pausing between reads

  std::thread srv([&]() {
    int c = ::accept(server.listen_fd, nullptr, nullptr);
    if (c >= 0)
    {
      run_ack_policy_peer(c, policy, received, acked, mutex, stop);
      ::close(c);
    }
  });

  ForwarderConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = server.port;
  cfg.write_timeout = std::chrono::milliseconds(20);  // shorter than the peer's drain cadence
  Forwarder forwarder(cfg);

  nlohmann::json big;
  big["blob"] = std::string(200000, 'x');
  Record big_record{ "dc.test", 1700000000ULL, 0u, big };

  bool saw_backpressure = false;
  for (int i = 0; i < 30 && !saw_backpressure; ++i)
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
      }
    }
  }

  // EXPECT (not ASSERT): `srv` is a joinable std::thread still running run_ack_policy_peer,
  // and an early return here without stop.store()+srv.join() would call std::terminate()
  // on the way out, taking the whole test binary down instead of just failing this test.
  EXPECT_TRUE(saw_backpressure) << "a peer that drains slowly should eventually stall a write past write_timeout";
  if (saw_backpressure)
  {
    EXPECT_TRUE(forwarder.is_connected()) << "backpressure must not drop the connection — the peer may just be slow";
  }

  stop.store(true);
  srv.join();
}

// #366: an ack that doesn't arrive within ack_timeout triggers a resend carrying the
// SAME chunk id (UnackedRecordResendsAfterReconnect above covers the reconnect trigger;
// this covers the timeout trigger). If the original, merely-delayed ack then arrives
// after the resend, the peer has seen the chunk twice — a wire duplicate, not a loss —
// and the window must still converge to zero rather than getting stuck or amplifying.
TEST(Forwarder, ResendAfterAckTimeoutProducesDuplicateThenConverges)
{
  MockServer server;
  std::atomic<bool> stop{ false };
  std::vector<std::string> received, acked;
  std::mutex mutex;
  AckPolicy policy;
  policy.ack_delay = std::chrono::milliseconds(300);  // longer than ack_timeout below

  std::thread srv([&]() {
    int c = ::accept(server.listen_fd, nullptr, nullptr);
    if (c >= 0)
    {
      run_ack_policy_peer(c, policy, received, acked, mutex, stop);
      ::close(c);
    }
  });

  ForwarderConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = server.port;
  cfg.ack_timeout = std::chrono::milliseconds(100);
  Forwarder forwarder(cfg);

  forwarder.send(make_record("dc.test", R"({"n": 1})"));

  // Poll past ack_timeout so the Forwarder resends the still-unacked record at least
  // once before the peer's 300ms delayed ack for the ORIGINAL send has a chance to land.
  bool resent = poll_until(std::chrono::milliseconds(500), [&]() {
    forwarder.poll();
    std::lock_guard<std::mutex> lock(mutex);
    return received.size() >= 2;
  });
  // EXPECT (not ASSERT): see SlowDrainingPeerIsBackpressureWithConnectionRetained above —
  // `srv` must always be stopped and joined before this test function can return.
  EXPECT_TRUE(resent) << "an unacked record past ack_timeout should be resent, duplicating it on the wire";
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (received.size() >= 2)
    {
      EXPECT_EQ(received[0], received[1]) << "the resend must carry the same chunk id as the original send";
    }
  }

  bool converged = poll_until(std::chrono::seconds(2), [&]() {
    forwarder.poll();
    return forwarder.unacked_count() == 0;
  });
  stop.store(true);
  srv.join();
  EXPECT_TRUE(converged) << "the duplicate ack(s) for the same chunk id must still clear the window, not wedge it";
}

// #366: handle_ack_object() finds the acked entry by chunk id via linear search, not by
// position — this pins that an ack arriving out of send order evicts only the record it
// actually names, leaving the other's exact byte count in the window.
TEST(Forwarder, OutOfOrderAckEvictsOnlyTheAckedRecord)
{
  MockServer server;
  std::atomic<bool> ack_a{ false };

  std::thread srv([&]() {
    int c = ::accept(server.listen_fd, nullptr, nullptr);

    // Both frames may arrive in a single recv() (they're written back to back with
    // nothing forcing a boundary), so parse incrementally with a real unpacker rather
    // than assuming one recv() call == one frame.
    msgpack::unpacker unpacker;
    std::string chunk_a, chunk_b;
    while (chunk_a.empty() || chunk_b.empty())
    {
      unpacker.reserve_buffer(4096);
      ssize_t n = ::recv(c, unpacker.buffer(), 4096, 0);
      if (n <= 0)
      {
        return;
      }
      unpacker.buffer_consumed(static_cast<std::size_t>(n));
      msgpack::object_handle oh;
      while (unpacker.next(oh))
      {
        msgpack::object top = oh.get();
        std::string chunk_id;
        if (top.type == msgpack::type::ARRAY && top.via.array.size >= 3 &&
            top.via.array.ptr[2].type == msgpack::type::MAP)
        {
          const msgpack::object& option = top.via.array.ptr[2];
          for (std::uint32_t i = 0; i < option.via.map.size; ++i)
          {
            if (option.via.map.ptr[i].key.as<std::string>() == "chunk")
            {
              chunk_id = option.via.map.ptr[i].val.as<std::string>();
            }
          }
        }
        if (chunk_a.empty())
        {
          chunk_a = chunk_id;
        }
        else if (chunk_b.empty())
        {
          chunk_b = chunk_id;
        }
      }
    }

    // Ack B first — out of send order.
    msgpack::sbuffer ack_buf;
    msgpack::packer<msgpack::sbuffer> pk(ack_buf);
    pk.pack_map(1);
    pk.pack(std::string("ack"));
    pk.pack(chunk_b);
    ::send(c, ack_buf.data(), ack_buf.size(), 0);

    while (!ack_a.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    msgpack::sbuffer ack_buf_a;
    msgpack::packer<msgpack::sbuffer> pk_a(ack_buf_a);
    pk_a.pack_map(1);
    pk_a.pack(std::string("ack"));
    pk_a.pack(chunk_a);
    ::send(c, ack_buf_a.data(), ack_buf_a.size(), 0);
    ::close(c);
  });

  ForwarderConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = server.port;
  Forwarder forwarder(cfg);

  // Different payload sizes so record A's and record B's frame byte counts differ,
  // making the eviction unambiguous from unacked_bytes() alone.
  Record record_a{ "dc.test", 1700000000ULL, 0u, nlohmann::json::parse(R"({"n": 1})") };
  nlohmann::json big;
  big["blob"] = std::string(500, 'x');
  Record record_b{ "dc.test", 1700000000ULL, 0u, big };

  forwarder.send(record_a);
  forwarder.send(record_b);
  // EXPECT (not ASSERT) throughout: `srv` is a joinable std::thread whose lambda blocks
  // on `ack_a`, and an ASSERT's early return without joining it would call
  // std::terminate() on the way out of this test function, taking the whole binary down
  // instead of just failing this one test.
  EXPECT_EQ(forwarder.unacked_count(), 2u);

  // A 22-char placeholder chunk id: generate_chunk_id() base64-encodes 16 random bytes
  // without padding (ceil(16*8/6) = 22 chars), always this length regardless of the
  // random content, and msgpack's fixstr encoding costs the same regardless of a
  // string's actual characters — so this reproduces the exact frame byte count
  // Forwarder used for its real (random) chunk id.
  const std::string placeholder_chunk_id(22, '0');
  const std::size_t expected_a_bytes = Forwarder::frame(record_a, placeholder_chunk_id).size();

  bool b_evicted = poll_until(std::chrono::seconds(5), [&]() {
    forwarder.poll();
    return forwarder.unacked_count() == 1;
  });
  EXPECT_TRUE(b_evicted) << "the out-of-order ack for B should evict exactly B";
  if (b_evicted)
  {
    EXPECT_EQ(forwarder.unacked_bytes(), expected_a_bytes) << "record A, not B, should remain in the window";
  }

  ack_a.store(true);
  poll_until(std::chrono::seconds(5), [&]() {
    forwarder.poll();
    return forwarder.unacked_count() == 0;
  });
  srv.join();
}
