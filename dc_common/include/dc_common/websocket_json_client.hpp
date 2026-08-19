// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_COMMON_WEBSOCKET_JSON_CLIENT_HPP_
#define DC_COMMON_WEBSOCKET_JSON_CLIENT_HPP_

#include <algorithm>
#include <atomic>
#include <boost/asio/connect.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace dc_common
{

/// Connection state reported to a WebSocketJsonClient's state handler.
enum class WebSocketState
{
  Disconnected,
  Connecting,
  Connected,
};

/// Tuning for WebSocketJsonClient's reconnect backoff. A free struct (not nested in
/// WebSocketJsonClient, though it's used as `WebSocketJsonClient::Options` there via a type
/// alias): a nested struct's own default member initializers aren't usable in a default argument
/// of the enclosing class's member function -- GCC/Clang both reject that ordering.
struct WebSocketJsonClientOptions
{
  /// Delay before the first reconnect attempt after a drop.
  std::chrono::milliseconds initial_backoff{ 500 };
  /// Reconnect delay never grows past this, however many attempts fail in a row.
  std::chrono::milliseconds max_backoff{ 30000 };
  /// Multiplier applied to the backoff after each failed connect/handshake/read.
  double backoff_multiplier{ 2.0 };
};

/**
 * @class dc_common::WebSocketJsonClient
 * @brief Connects to a `ws://` URL, sends/receives newline-free JSON text frames, and reconnects
 * with exponential backoff on a dropped connection -- no ROS dependency, transport only (#390).
 *
 * Every existing Measurement subscribes to something inside the robot's own ROS graph, where DDS
 * handles reconnection for free; this is what a Measurement reaching outside that graph (e.g. the
 * Open-RMF task-state API, #391) uses instead. A Measurement plugin owns one instance, calling
 * connect() from onConfigure() and disconnect() from onCleanup() -- the same shape every other
 * Measurement's create_subscription()/reset() pair already follows (see dc_measurements::Measurement).
 *
 * Runs its own io_context on a background thread so connect()/send()/disconnect() never block the
 * caller. onMessage/onStateChange/onError handlers fire on that background thread -- not the
 * caller's -- so a handler touching ROS state must marshal itself onto the node's executor. Message,
 * state and error handlers should be set once, before the first connect().
 */
class WebSocketJsonClient
{
public:
  using MessageHandler = std::function<void(const nlohmann::json&)>;
  using StateHandler = std::function<void(WebSocketState)>;
  using ErrorHandler = std::function<void(const std::string&)>;
  using Options = WebSocketJsonClientOptions;

  explicit WebSocketJsonClient(Options options = Options{}) : options_(options)
  {
  }

  ~WebSocketJsonClient()
  {
    disconnect();
  }

  WebSocketJsonClient(const WebSocketJsonClient&) = delete;
  WebSocketJsonClient& operator=(const WebSocketJsonClient&) = delete;

  void onMessage(MessageHandler handler)
  {
    message_handler_ = std::move(handler);
  }

  void onStateChange(StateHandler handler)
  {
    state_handler_ = std::move(handler);
  }

  void onError(ErrorHandler handler)
  {
    error_handler_ = std::move(handler);
  }

  /**
   * @brief Start connecting to `url` (`ws://host[:port][/path]`) and keep reconnecting with
   * exponential backoff on any drop until disconnect() is called. One connection at a time: call
   * disconnect() first before connecting to a different URL on the same instance.
   */
  void connect(const std::string& url)
  {
    parsed_url_ = parseUrl(url);
    stopping_ = false;
    current_backoff_ = options_.initial_backoff;
    // Bumped here (not just in doConnect()) so a completion from a *previous* connect()/
    // disconnect() cycle -- e.g. a resolver lookup still in flight when disconnect() was called,
    // whose handler asio only gets around to delivering after this new run() starts -- reads a
    // stale value and is ignored below, instead of racing this session's own connection attempt.
    ++generation_;
    io_thread_ = std::thread([this] { runIoThread(); });
  }

  /// Stops reconnection and closes any open connection. Idempotent; safe to call from any thread.
  void disconnect()
  {
    if (!io_thread_.joinable())
    {
      return;
    }
    stopping_ = true;
    ++generation_;
    // Posted rather than done inline: ws_/reconnect_timer_ are touched only from io_thread_, so
    // tearing them down has to happen there too, then stop() the io_context from that same handler
    // so run() cannot return before the close has actually been issued.
    boost::asio::post(ioc_, [this] {
      closeStream();
      ioc_.stop();
    });
    io_thread_.join();
    ioc_.restart();
    setState(WebSocketState::Disconnected);
  }

  /// Serializes `message` and sends it as a text frame on the current connection; silently dropped
  /// if not connected.
  void send(const nlohmann::json& message)
  {
    if (state() != WebSocketState::Connected)
    {
      return;
    }
    auto text = std::make_shared<std::string>(message.dump());
    boost::asio::post(ioc_, [this, text] {
      if (!ws_ || !ws_->is_open())
      {
        return;
      }
      ws_->async_write(boost::asio::buffer(*text), [this](boost::beast::error_code ec, std::size_t) {
        if (ec)
        {
          reportError("write failed: " + ec.message());
        }
      });
    });
  }

  WebSocketState state() const
  {
    return state_.load();
  }

private:
  struct ParsedUrl
  {
    std::string host;
    std::string port;
    std::string target;
  };

  /// Minimal `ws://host[:port][/path]` parser. `wss://` (TLS) is out of scope here, matching this
  /// ticket's transport-only remit -- adding it later only changes doConnect()'s stream type.
  static ParsedUrl parseUrl(const std::string& url)
  {
    static const std::string scheme_prefix = "ws://";
    if (url.rfind(scheme_prefix, 0) != 0)
    {
      throw std::invalid_argument{ "WebSocketJsonClient only supports ws:// URLs, got: " + url };
    }
    const std::string rest = url.substr(scheme_prefix.size());
    const auto slash = rest.find('/');
    const std::string authority = slash == std::string::npos ? rest : rest.substr(0, slash);
    const std::string target = slash == std::string::npos ? "/" : rest.substr(slash);
    const auto colon = authority.find(':');

    ParsedUrl parsed;
    if (colon == std::string::npos)
    {
      parsed.host = authority;
      parsed.port = "80";
    }
    else
    {
      parsed.host = authority.substr(0, colon);
      parsed.port = authority.substr(colon + 1);
    }
    parsed.target = target;

    if (parsed.host.empty())
    {
      throw std::invalid_argument{ "WebSocketJsonClient: empty host in URL: " + url };
    }
    return parsed;
  }

  void runIoThread()
  {
    doConnect();
    ioc_.run();
  }

  void doConnect()
  {
    // Captured by every completion handler below and compared against generation_ when it fires:
    // a handler whose generation has fallen behind belongs to a connect()/disconnect() cycle that
    // is no longer current (e.g. a resolve still in flight when disconnect() ran) and must be a
    // no-op rather than act on -- or replace -- the current attempt's ws_.
    const auto gen = generation_.load();
    setState(WebSocketState::Connecting);
    ws_ = std::make_unique<boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>(ioc_);
    resolver_.async_resolve(
        parsed_url_.host, parsed_url_.port,
        [this, gen](boost::beast::error_code ec, const boost::asio::ip::tcp::resolver::results_type& results) {
          if (gen != generation_.load())
          {
            return;
          }
          if (ec)
          {
            reportError("resolve failed: " + ec.message());
            scheduleReconnect(gen);
            return;
          }
          boost::asio::async_connect(ws_->next_layer(), results,
                                     [this, gen](boost::beast::error_code connect_ec,
                                                 const boost::asio::ip::tcp::endpoint&) { onConnect(gen, connect_ec); });
        });
  }

  void onConnect(std::uint64_t gen, boost::beast::error_code ec)
  {
    if (gen != generation_.load())
    {
      return;
    }
    if (ec)
    {
      reportError("connect failed: " + ec.message());
      scheduleReconnect(gen);
      return;
    }
    ws_->async_handshake(parsed_url_.host + ":" + parsed_url_.port, parsed_url_.target,
                         [this, gen](boost::beast::error_code handshake_ec) { onHandshake(gen, handshake_ec); });
  }

  void onHandshake(std::uint64_t gen, boost::beast::error_code ec)
  {
    if (gen != generation_.load())
    {
      return;
    }
    if (ec)
    {
      reportError("handshake failed: " + ec.message());
      scheduleReconnect(gen);
      return;
    }
    ws_->text(true);
    current_backoff_ = options_.initial_backoff;
    setState(WebSocketState::Connected);
    doRead(gen);
  }

  void doRead(std::uint64_t gen)
  {
    ws_->async_read(buffer_, [this, gen](boost::beast::error_code ec, std::size_t) {
      if (gen != generation_.load())
      {
        return;
      }
      if (ec)
      {
        // stopping_ means this read failed because disconnect() closed the stream out from under
        // it -- expected, not a drop, and reconnection must not be scheduled behind disconnect()'s
        // back.
        if (!stopping_)
        {
          reportError("read failed: " + ec.message());
          setState(WebSocketState::Disconnected);
          scheduleReconnect(gen);
        }
        return;
      }
      const auto text = boost::beast::buffers_to_string(buffer_.data());
      buffer_.consume(buffer_.size());
      try
      {
        const auto json_message = nlohmann::json::parse(text);
        if (message_handler_)
        {
          message_handler_(json_message);
        }
      }
      catch (const nlohmann::json::parse_error& e)
      {
        reportError(std::string("received non-JSON message: ") + e.what());
      }
      doRead(gen);
    });
  }

  void scheduleReconnect(std::uint64_t gen)
  {
    if (stopping_)
    {
      return;
    }
    setState(WebSocketState::Disconnected);
    reconnect_timer_.expires_after(current_backoff_);
    reconnect_timer_.async_wait([this, gen](boost::beast::error_code ec) {
      if (ec || stopping_ || gen != generation_.load())
      {
        return;
      }
      current_backoff_ = std::min(
          std::chrono::milliseconds(static_cast<std::int64_t>(current_backoff_.count() * options_.backoff_multiplier)),
          options_.max_backoff);
      doConnect();
    });
  }

  /// Only ever called from io_thread_ (directly, or via post() from disconnect()).
  ///
  /// Closes the underlying TCP layer directly rather than attempting a graceful websocket close
  /// handshake (`ws_->close()`): doRead() keeps exactly one async_read perpetually outstanding on
  /// ws_ while connected, and Beast does not support a second read-family operation (which a
  /// synchronous close()'s wait-for-the-peer's-close-frame amounts to) starting while one is
  /// already pending on the same stream -- verified empirically, it silently never completes a
  /// real handshake. Closing the transport instead completes the outstanding async_read (with an
  /// error, which doRead() already treats as a no-op once stopping_ is set) immediately, safely,
  /// same-thread.
  void closeStream()
  {
    reconnect_timer_.cancel();
    resolver_.cancel();
    if (!ws_)
    {
      return;
    }
    boost::beast::error_code ec;
    ws_->next_layer().close(ec);
  }

  void setState(WebSocketState new_state)
  {
    state_.store(new_state);
    if (state_handler_)
    {
      state_handler_(new_state);
    }
  }

  void reportError(const std::string& message)
  {
    if (error_handler_)
    {
      error_handler_(message);
    }
  }

  Options options_;
  ParsedUrl parsed_url_;
  std::atomic<bool> stopping_{ true };
  std::atomic<WebSocketState> state_{ WebSocketState::Disconnected };
  std::chrono::milliseconds current_backoff_{ 500 };
  /// Incremented on every connect()/disconnect(); see doConnect()'s comment on `gen`.
  std::atomic<std::uint64_t> generation_{ 0 };

  boost::asio::io_context ioc_;
  boost::asio::ip::tcp::resolver resolver_{ ioc_ };
  boost::asio::steady_timer reconnect_timer_{ ioc_ };
  std::unique_ptr<boost::beast::websocket::stream<boost::asio::ip::tcp::socket>> ws_;
  boost::beast::flat_buffer buffer_;
  std::thread io_thread_;

  MessageHandler message_handler_;
  StateHandler state_handler_;
  ErrorHandler error_handler_;
};

}  // namespace dc_common

#endif  // DC_COMMON_WEBSOCKET_JSON_CLIENT_HPP_
