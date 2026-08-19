// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Unit tests for dc_common::WebSocketJsonClient (#390). No ROS node, no rclcpp::init(): the
// fixture is a trivial synchronous websocket echo server bound to an ephemeral loopback port, per
// #360's ROS-free-unit precedent.

#include "dc_common/websocket_json_client.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/websocket.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

using dc_common::WebSocketJsonClient;
using dc_common::WebSocketState;

namespace
{

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

// Same idiom as dc_bridge/test/forwarder_test.cpp's poll_until(): a wall-clock deadline rather
// than a fixed iteration count, so scheduler contention or a slow build shrinks nothing but the
// margin.
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

// A trivial websocket echo server: binds :0 on loopback, accepts connections one at a time on a
// background thread, and echoes every text frame it reads back verbatim. dropConnection() force-
// closes whichever connection is current, standing in for a network drop; the accept loop then
// takes the next incoming connection, so a reconnecting client succeeds.
//
// Entirely async, driven by ioc_.run() on thread_ -- not a synchronous accept()/read() loop.
// Boost.Asio only guarantees that closing a socket/acceptor safely and promptly cancels an
// operation *pending on that same io_context*, regardless of which thread calls close(); a raw
// blocking syscall in another thread has no such guarantee (closing an fd out from under a thread
// already blocked in accept()/read() on it does not reliably unblock that call on Linux) --
// verified empirically here, it hangs. So dropConnection()/stop() below post their close() calls
// onto ioc_ instead of calling it inline from the test thread, exactly like
// WebSocketJsonClient::disconnect() already does for the same reason.
class EchoServer
{
public:
  EchoServer() : acceptor_(ioc_, tcp::endpoint(tcp::v4(), 0))
  {
    doAccept();
    thread_ = std::thread([this] { ioc_.run(); });
  }

  ~EchoServer()
  {
    stop();
  }

  unsigned short port() const
  {
    return acceptor_.local_endpoint().port();
  }

  void dropConnection()
  {
    asio::post(ioc_, [this] {
      if (current_ws_ && current_ws_->is_open())
      {
        beast::error_code ec;
        current_ws_->next_layer().close(ec);
      }
    });
  }

  void stop()
  {
    if (stopped_.exchange(true))
    {
      return;
    }
    asio::post(ioc_, [this] {
      boost::system::error_code ec;
      acceptor_.close(ec);
      if (current_ws_ && current_ws_->is_open())
      {
        beast::error_code wec;
        current_ws_->next_layer().close(wec);
      }
      ioc_.stop();
    });
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

private:
  // current_ws_ is only ever touched from thread_ (directly, or via asio::post() from
  // dropConnection()/stop() above), so it needs no lock.
  void doAccept()
  {
    acceptor_.async_accept([this](boost::system::error_code ec, tcp::socket socket) {
      if (ec)
      {
        // acceptor_ closed (stop()) or a real error either way there's nothing more to accept.
        return;
      }
      auto ws = std::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
      current_ws_ = ws;
      ws->async_accept([this, ws](beast::error_code handshake_ec) {
        if (handshake_ec)
        {
          if (!stopped_)
          {
            doAccept();
          }
          return;
        }
        ws->text(true);
        doRead(ws);
      });
    });
  }

  void doRead(const std::shared_ptr<websocket::stream<tcp::socket>>& ws)
  {
    auto buffer = std::make_shared<beast::flat_buffer>();
    ws->async_read(*buffer, [this, ws, buffer](beast::error_code ec, std::size_t) {
      if (ec)
      {
        // This connection is done (closed by the client, or dropConnection()); the next one --
        // possibly a reconnecting client -- gets its own accept.
        if (!stopped_)
        {
          doAccept();
        }
        return;
      }
      ws->async_write(buffer->data(), [this, ws, buffer](beast::error_code write_ec, std::size_t) {
        if (write_ec)
        {
          if (!stopped_)
          {
            doAccept();
          }
          return;
        }
        doRead(ws);
      });
    });
  }

  asio::io_context ioc_;
  tcp::acceptor acceptor_;
  std::thread thread_;
  std::atomic<bool> stopped_{ false };
  std::shared_ptr<websocket::stream<tcp::socket>> current_ws_;
};

// Records every state transition and message a client reports, guarded by one mutex so a test can
// poll it from the gtest thread while the client's handlers fire on its own IO thread.
struct ClientObserver
{
  std::mutex mutex;
  std::vector<WebSocketState> states;
  std::vector<nlohmann::json> messages;
  std::vector<std::string> errors;

  void wire(WebSocketJsonClient& client)
  {
    client.onStateChange([this](WebSocketState s) {
      std::lock_guard<std::mutex> lock(mutex);
      states.push_back(s);
    });
    client.onMessage([this](const nlohmann::json& msg) {
      std::lock_guard<std::mutex> lock(mutex);
      messages.push_back(msg);
    });
    client.onError([this](const std::string& msg) {
      std::lock_guard<std::mutex> lock(mutex);
      errors.push_back(msg);
    });
  }

  bool sawState(WebSocketState s)
  {
    std::lock_guard<std::mutex> lock(mutex);
    return std::find(states.begin(), states.end(), s) != states.end();
  }

  std::size_t stateCount(WebSocketState s)
  {
    std::lock_guard<std::mutex> lock(mutex);
    return static_cast<std::size_t>(std::count(states.begin(), states.end(), s));
  }

  std::size_t messageCount()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return messages.size();
  }

  nlohmann::json lastMessage()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return messages.back();
  }
};

std::string wsUrl(const EchoServer& server)
{
  return "ws://127.0.0.1:" + std::to_string(server.port()) + "/";
}

}  // namespace

TEST(WebSocketJsonClient, RejectsNonWebSocketUrl)
{
  WebSocketJsonClient client;
  EXPECT_THROW(client.connect("http://127.0.0.1:9"), std::invalid_argument);
}

TEST(WebSocketJsonClient, RejectsUrlWithEmptyHost)
{
  WebSocketJsonClient client;
  EXPECT_THROW(client.connect("ws:///path"), std::invalid_argument);
}

TEST(WebSocketJsonClient, ConnectsSendsAndReceivesJson)
{
  EchoServer server;
  ClientObserver observer;
  WebSocketJsonClient client;
  observer.wire(client);

  client.connect(wsUrl(server));
  ASSERT_TRUE(poll_until(std::chrono::milliseconds(2000), [&] { return client.state() == WebSocketState::Connected; }));

  const nlohmann::json request{ { "op", "subscribe" }, { "task_id", "abc-123" } };
  client.send(request);

  ASSERT_TRUE(poll_until(std::chrono::milliseconds(2000), [&] { return observer.messageCount() >= 1; }));
  EXPECT_EQ(observer.lastMessage(), request);

  client.disconnect();
  EXPECT_EQ(client.state(), WebSocketState::Disconnected);
}

TEST(WebSocketJsonClient, ReconnectsAfterDroppedConnectionAndKeepsWorking)
{
  EchoServer server;
  ClientObserver observer;
  WebSocketJsonClient::Options options;
  options.initial_backoff = std::chrono::milliseconds(50);
  options.max_backoff = std::chrono::milliseconds(200);
  WebSocketJsonClient client(options);
  observer.wire(client);

  client.connect(wsUrl(server));
  ASSERT_TRUE(poll_until(std::chrono::milliseconds(2000), [&] { return client.state() == WebSocketState::Connected; }));

  server.dropConnection();

  // Reconnection: the client must observe the drop, then re-establish a *working* connection --
  // not just flip back to Connected, which a stale/half-open socket could also do. Proven by
  // sending after the second Connected and getting a real echo back, not just the state value.
  ASSERT_TRUE(
      poll_until(std::chrono::milliseconds(3000), [&] { return observer.stateCount(WebSocketState::Connected) >= 2; }));

  const nlohmann::json request{ { "after", "reconnect" } };
  client.send(request);
  ASSERT_TRUE(poll_until(std::chrono::milliseconds(2000), [&] { return observer.messageCount() >= 1; }));
  EXPECT_EQ(observer.lastMessage(), request);

  EXPECT_TRUE(observer.sawState(WebSocketState::Disconnected));

  client.disconnect();
}

TEST(WebSocketJsonClient, DisconnectStopsReconnectionAndIsIdempotent)
{
  EchoServer server;
  WebSocketJsonClient::Options options;
  options.initial_backoff = std::chrono::milliseconds(50);
  options.max_backoff = std::chrono::milliseconds(100);
  WebSocketJsonClient client(options);

  client.connect(wsUrl(server));
  ASSERT_TRUE(poll_until(std::chrono::milliseconds(2000), [&] { return client.state() == WebSocketState::Connected; }));

  client.disconnect();
  EXPECT_EQ(client.state(), WebSocketState::Disconnected);

  // No crash/hang, and the state stays put: a reconnect loop still running behind disconnect()'s
  // back would eventually flip this back to Connected/Connecting on its own.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(client.state(), WebSocketState::Disconnected);

  client.disconnect();
  EXPECT_EQ(client.state(), WebSocketState::Disconnected);
}

TEST(WebSocketJsonClient, DisconnectBeforeAnyConnectIsSafe)
{
  WebSocketJsonClient client;
  EXPECT_EQ(client.state(), WebSocketState::Disconnected);
  client.disconnect();
  EXPECT_EQ(client.state(), WebSocketState::Disconnected);
}

TEST(WebSocketJsonClient, CanReconnectToANewUrlAfterDisconnect)
{
  EchoServer server_a;
  WebSocketJsonClient client;

  client.connect(wsUrl(server_a));
  ASSERT_TRUE(poll_until(std::chrono::milliseconds(2000), [&] { return client.state() == WebSocketState::Connected; }));
  client.disconnect();
  ASSERT_EQ(client.state(), WebSocketState::Disconnected);

  EchoServer server_b;
  ClientObserver observer;
  observer.wire(client);
  client.connect(wsUrl(server_b));
  ASSERT_TRUE(poll_until(std::chrono::milliseconds(2000), [&] { return client.state() == WebSocketState::Connected; }));

  const nlohmann::json request{ { "server", "b" } };
  client.send(request);
  ASSERT_TRUE(poll_until(std::chrono::milliseconds(2000), [&] { return observer.messageCount() >= 1; }));
  EXPECT_EQ(observer.lastMessage(), request);

  client.disconnect();
}
