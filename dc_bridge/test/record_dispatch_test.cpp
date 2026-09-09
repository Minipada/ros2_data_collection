// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// The per-Record dispatch (#494): the enqueue-vs-forward decision, the parse-or-wrap
// rule, and the stamp/envelope assembly, against a real IntentQueue and a loopback
// shipper ingest protocol peer — no ROS node.
#include "dc_bridge/record_dispatch.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <msgpack.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "dc_bridge/uploader/intent_queue.hpp"

using namespace dc_bridge;
using namespace dc_bridge::uploader;  // NOLINT
using nlohmann::json;

namespace
{

std::atomic<int> g_counter{ 0 };

std::filesystem::path unique_dir()
{
  return std::filesystem::temp_directory_path() /
         ("dc_record_dispatch_test_" + std::to_string(::getpid()) + "_" + std::to_string(g_counter++));
}

IncomingRecord make_incoming(const std::string& topic, const std::string& data, std::int32_t secs = 1700000000,
                             std::uint32_t nanos = 123456789)
{
  IncomingRecord in;
  in.topic = topic;
  in.stamp_secs = secs;
  in.stamp_nanos = nanos;
  in.data = data;
  return in;
}

// The EventTime extension's wire bytes (msgpack ext type 0x00, 8-byte big-endian body),
// built independently of the packing code so the test pins the bytes (see forwarder_test).
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

// A loopback peer: binds port 0, accepts one connection in a background thread, and
// captures the first frame's bytes. Bounded timeouts, so a test whose dispatch never
// sends still tears down.
struct CapturedFrame
{
  int listen_fd{ -1 };
  std::uint16_t port{ 0 };
  timeval rcv_timeout{ 0, 2000000 };  // 2s — also bounds accept() on Linux
  std::string bytes;
  std::mutex mutex;
  std::atomic<bool> got_frame{ false };
  std::thread thread;

  CapturedFrame()
  {
    listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    ::setsockopt(listen_fd, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sa.sin_port = 0;
    ::bind(listen_fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa));
    ::listen(listen_fd, 4);
    socklen_t len = sizeof(sa);
    ::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&sa), &len);
    port = ntohs(sa.sin_port);

    thread = std::thread([this]() {
      int fd = ::accept(listen_fd, nullptr, nullptr);
      if (fd < 0)
      {
        return;
      }
      ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));
      char buf[8192];
      ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
      if (n > 0)
      {
        std::lock_guard<std::mutex> lock(mutex);
        bytes.assign(buf, static_cast<std::size_t>(n));
        got_frame.store(true);
      }
      ::close(fd);
    });
  }
  ~CapturedFrame()
  {
    thread.join();
    if (listen_fd >= 0)
      ::close(listen_fd);
  }

  std::string frame()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return bytes;
  }
};

// Hold the returned handle while using anything decoded from it: a msgpack::object points
// into the handle's own zone.
msgpack::object_handle unpack_frame(const std::string& frame)
{
  return msgpack::unpack(frame.data(), frame.size());
}

// [tag, [[time, record]], option] -> the record map, i.e. what the Shipper parses.
msgpack::object wire_record_map(const msgpack::object& top)
{
  EXPECT_EQ(top.type, msgpack::type::ARRAY);
  EXPECT_EQ(top.via.array.size, 3u);
  msgpack::object entries = top.via.array.ptr[1];
  EXPECT_EQ(entries.via.array.size, 1u);
  msgpack::object entry = entries.via.array.ptr[0];  // [time, record]
  EXPECT_EQ(entry.via.array.size, 2u);
  return entry.via.array.ptr[1];
}

std::string wire_tag(const std::string& frame)
{
  msgpack::object_handle oh = unpack_frame(frame);
  return oh.get().via.array.ptr[0].as<std::string>();
}

std::string wire_event_time(const std::string& frame)
{
  msgpack::object_handle oh = unpack_frame(frame);
  msgpack::object entry = oh.get().via.array.ptr[1].via.array.ptr[0];
  EXPECT_EQ(entry.type, msgpack::type::EXT);
  return std::string(entry.via.ext.ptr, static_cast<std::size_t>(entry.via.ext.size));
}

// The "message" value pack_record_map wraps a non-map payload in, or nullptr when the
// wire record is not exactly a one-key {"message": ...} map.
const msgpack::object* wire_message_value(const msgpack::object& record)
{
  if (record.type != msgpack::type::MAP || record.via.map.size != 1 ||
      record.via.map.ptr[0].key.as<std::string>() != "message")
  {
    return nullptr;
  }
  return &record.via.map.ptr[0].val;
}

// The peer thread reads concurrently with send(): poll for the frame rather than
// assuming it was observed the instant dispatch() returned.
bool frame_arrived_within(CapturedFrame& peer, std::chrono::milliseconds budget)
{
  const auto deadline = std::chrono::steady_clock::now() + budget;
  while (!peer.got_frame.load() && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return peer.got_frame.load();
}

bool wait_for_frame(CapturedFrame& peer)
{
  return frame_arrived_within(peer, std::chrono::seconds(2));
}

// Inverted, for the "dispatch() sent nothing" assertions: a bounded window in which a
// frame it did put on the wire would already have been observed.
bool no_frame_within(CapturedFrame& peer)
{
  return !frame_arrived_within(peer, std::chrono::milliseconds(200));
}

// A dispatcher over a real intent queue and a real Forwarder. `want_peer` decides whether
// the Shipper is a listening loopback peer or a port nothing accepts on, so both
// "nothing was sent" and "the send failed" are observable.
struct Harness
{
  std::filesystem::path queue_dir;
  std::unique_ptr<IntentQueue> queue;
  std::unique_ptr<CapturedFrame> peer;
  std::unique_ptr<Forwarder> forwarder;
  std::mutex forwarder_mutex;
  std::vector<std::string> warnings;
  std::unique_ptr<RecordDispatcher> dispatcher;

  Harness(RecordDispatcher::Topics topics, bool want_queue = true, bool want_peer = true) : queue_dir(unique_dir())
  {
    if (want_queue)
    {
      std::filesystem::create_directories(queue_dir);
      queue = std::make_unique<IntentQueue>(queue_dir.string());
    }

    ForwarderConfig fcfg;
    fcfg.host = "127.0.0.1";
    if (want_peer)
    {
      peer = std::make_unique<CapturedFrame>();
      fcfg.port = peer->port;
    }
    else
    {
      fcfg.port = 1;  // nothing listening
      fcfg.connect_timeout = std::chrono::milliseconds(200);
    }
    forwarder = std::make_unique<Forwarder>(fcfg);

    RecordDispatcher::Deps deps;
    deps.intent_queue = queue.get();
    deps.forwarder = forwarder.get();
    deps.forwarder_mutex = &forwarder_mutex;
    deps.on_warning = [this](const std::string& msg) { warnings.push_back(msg); };
    dispatcher = std::make_unique<RecordDispatcher>(std::move(topics), deps);
  }

  ~Harness()
  {
    // The dispatcher borrows everything else here; it must go first.
    dispatcher.reset();
    forwarder.reset();
    peer.reset();
    queue.reset();
    std::error_code ec;
    std::filesystem::remove_all(queue_dir, ec);
  }
};

}  // namespace

TEST(RecordDispatch, SubscribedTopicsIsTheDedupedUnionOfBothLists)
{
  RecordDispatcher::Topics topics;
  topics.records = { "/b", "/a" };
  topics.files = { "/c", "/b" };
  Harness h(std::move(topics));

  const std::vector<std::string> want = { "/a", "/b", "/c" };
  EXPECT_EQ(h.dispatcher->subscribed_topics(), want);
}

TEST(RecordDispatch, FilesTopicRecordIsEnqueuedAndNotForwarded)
{
  RecordDispatcher::Topics topics;
  topics.files = { "/dc/camera/image" };
  Harness h(std::move(topics));

  h.dispatcher->dispatch(make_incoming("/dc/camera/image", R"({"path": "/files/img.jpg"})"));

  ASSERT_EQ(h.queue->size(), 1u);
  const Intent intent = h.queue->pending().at(0);
  EXPECT_EQ(intent.tag, "dc.camera.image");
  EXPECT_EQ(intent.payload, json::parse(R"({"path": "/files/img.jpg"})"));

  // dispatch() is synchronous, so a frame it did send would be observed within this
  // window; nothing was.
  EXPECT_TRUE(no_frame_within(*h.peer));
  EXPECT_FALSE(h.forwarder->is_connected());
  EXPECT_TRUE(h.warnings.empty());
}

TEST(RecordDispatch, RecordsTopicRecordIsForwardedAndNotEnqueued)
{
  RecordDispatcher::Topics topics;
  topics.records = { "/dc/measurement/uptime" };
  Harness h(std::move(topics));

  h.dispatcher->dispatch(make_incoming("/dc/measurement/uptime", R"({"uptime_s": 42})"));

  EXPECT_EQ(h.queue->size(), 0u);
  EXPECT_TRUE(h.warnings.empty());
  ASSERT_TRUE(wait_for_frame(*h.peer));

  const std::string frame = h.peer->frame();
  EXPECT_EQ(wire_tag(frame), "dc.measurement.uptime");
  EXPECT_EQ(wire_event_time(frame), expected_event_time_bytes(1700000000, 123456789));

  msgpack::object_handle oh = unpack_frame(frame);
  msgpack::object record = wire_record_map(oh.get());
  ASSERT_EQ(record.type, msgpack::type::MAP);
  bool found = false;
  for (std::uint32_t i = 0; i < record.via.map.size; ++i)
  {
    if (record.via.map.ptr[i].key.as<std::string>() == "uptime_s")
    {
      EXPECT_EQ(record.via.map.ptr[i].val.as<std::uint64_t>(), 42u);
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST(RecordDispatch, TopicInBothListsIsEnqueuedAndForwarded)
{
  RecordDispatcher::Topics topics;
  topics.records = { "/dc/both" };
  topics.files = { "/dc/both" };
  Harness h(std::move(topics));

  h.dispatcher->dispatch(make_incoming("/dc/both", R"({"name": "both"})"));

  EXPECT_EQ(h.queue->size(), 1u);
  ASSERT_TRUE(wait_for_frame(*h.peer));
  EXPECT_EQ(wire_tag(h.peer->frame()), "dc.both");
}

TEST(RecordDispatch, NonJsonPayloadBecomesAJsonString)
{
  auto payload = RecordDispatcher::parse_payload("not json at all");
  EXPECT_TRUE(payload.is_string());
  EXPECT_EQ(payload.get<std::string>(), "not json at all");

  // Bytes that do parse pass through unchanged, whatever JSON type they are.
  EXPECT_EQ(RecordDispatcher::parse_payload("[1, 2, 3]"), json::parse("[1, 2, 3]"));
  EXPECT_EQ(RecordDispatcher::parse_payload(R"({"a": 1})"), json::parse(R"({"a": 1})"));
}

TEST(RecordDispatch, NonJsonPayloadIsLeftForPackRecordMapToWrap)
{
  RecordDispatcher::Topics topics;
  topics.records = { "/dc/measurement/raw" };
  Harness h(std::move(topics));

  h.dispatcher->dispatch(make_incoming("/dc/measurement/raw", "not json at all"));

  ASSERT_TRUE(wait_for_frame(*h.peer));
  msgpack::object_handle oh = unpack_frame(h.peer->frame());
  const msgpack::object* message = wire_message_value(wire_record_map(oh.get()));
  ASSERT_NE(message, nullptr);
  EXPECT_EQ(message->as<std::string>(), "not json at all");
}

TEST(RecordDispatch, NonJsonAndJsonStringPayloadsLandTheSameWay)
{
  RecordDispatcher::Topics topics;
  topics.records = { "/dc/measurement/raw" };
  Harness h(std::move(topics));

  // The same text, delivered as bytes that parse as a JSON string, must produce the very
  // same wire record — the parse-or-wrap step adds no second wrapping of its own.
  h.dispatcher->dispatch(make_incoming("/dc/measurement/raw", "\"not json at all\""));

  ASSERT_TRUE(wait_for_frame(*h.peer));
  msgpack::object_handle oh = unpack_frame(h.peer->frame());
  const msgpack::object* message = wire_message_value(wire_record_map(oh.get()));
  ASSERT_NE(message, nullptr);
  EXPECT_EQ(message->as<std::string>(), "not json at all");
}

TEST(RecordDispatch, ArrayPayloadIsNotCoercedIntoAnObject)
{
  RecordDispatcher::Topics topics;
  topics.records = { "/dc/measurement/array" };
  Harness h(std::move(topics));

  h.dispatcher->dispatch(make_incoming("/dc/measurement/array", "[1, 2, 3]"));

  ASSERT_TRUE(wait_for_frame(*h.peer));
  msgpack::object_handle oh = unpack_frame(h.peer->frame());
  const msgpack::object* message = wire_message_value(wire_record_map(oh.get()));
  ASSERT_NE(message, nullptr);
  ASSERT_EQ(message->type, msgpack::type::ARRAY);
  ASSERT_EQ(message->via.array.size, 3u);
  EXPECT_EQ(message->via.array.ptr[2].as<std::uint64_t>(), 3u);
}

TEST(RecordDispatch, NegativeStampSecondsClampToZeroAndNanosecondsSurvive)
{
  RecordDispatcher::Topics topics;
  topics.records = { "/dc/measurement/clock" };
  Harness h(std::move(topics));

  h.dispatcher->dispatch(make_incoming("/dc/measurement/clock", "{}", -5, 42));

  ASSERT_TRUE(wait_for_frame(*h.peer));
  EXPECT_EQ(wire_event_time(h.peer->frame()), expected_event_time_bytes(0, 42));
}

TEST(RecordDispatch, UnreachableShipperWarnsInsteadOfThrowing)
{
  RecordDispatcher::Topics topics;
  topics.records = { "/dc/measurement/uptime" };
  Harness h(std::move(topics), /*want_queue=*/true, /*want_peer=*/false);

  h.dispatcher->dispatch(make_incoming("/dc/measurement/uptime", "{}"));

  ASSERT_EQ(h.warnings.size(), 1u);
  EXPECT_NE(h.warnings.at(0).find("dc.measurement.uptime"), std::string::npos);
}
