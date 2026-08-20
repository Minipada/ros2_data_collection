// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Integration test for dc_measurements::MissionOpenRmf (#391): a real websocket server (a small
// Boost.Beast fixture, PushServer below -- distinct from dc_common's echo-only fixture, since this
// one needs to push server-initiated TaskState frames on demand) stands in for the bridge
// `websocket_url` would point at in a real deployment, and the plugin under test connects to it
// through the real dc_common::WebSocketJsonClient (#390), same as it would in production.

#include <gtest/gtest.h>

#include <atomic>
#include <boost/asio/connect.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/post.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <chrono>
#include <fstream>
#include <functional>
#include <future>
#include <memory>
#include <nlohmann/json-schema.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

namespace
{

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

// A websocket server that accepts one connection and pushes server-initiated text frames to it on
// demand (send()), as opposed to dc_common's test-only EchoServer which only ever echoes what a
// client sends. Entirely async, driven by ioc_.run() on thread_, same shape/reasoning as
// dc_common/test/websocket_json_client_test.cpp's EchoServer.
class PushServer
{
public:
  PushServer() : acceptor_(ioc_, tcp::endpoint(tcp::v4(), 0)), work_guard_(asio::make_work_guard(ioc_))
  {
    doAccept();
    thread_ = std::thread([this] { ioc_.run(); });
  }

  ~PushServer()
  {
    stop();
  }

  unsigned short port() const
  {
    return acceptor_.local_endpoint().port();
  }

  bool waitForConnection(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!connected_.load() && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return connected_.load();
  }

  /// Sends `message` as a single text frame on the current connection. Only meaningful after
  /// waitForConnection() has returned true; a no-op otherwise.
  void send(const nlohmann::json& message)
  {
    auto text = std::make_shared<std::string>(message.dump());
    std::promise<void> done;
    auto fut = done.get_future();
    asio::post(ioc_, [this, text, &done] {
      if (current_ws_ && current_ws_->is_open())
      {
        beast::error_code ec;
        current_ws_->write(asio::buffer(*text), ec);
      }
      done.set_value();
    });
    fut.wait();
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
      work_guard_.reset();
      ioc_.stop();
    });
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

private:
  void doAccept()
  {
    acceptor_.async_accept([this](boost::system::error_code ec, tcp::socket socket) {
      if (ec)
      {
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
        connected_.store(true);
      });
    });
  }

  asio::io_context ioc_;
  tcp::acceptor acceptor_;
  // Once a connection's handshake completes, this server issues no further async_accept and never
  // reads from the connection (server-push-only, unlike EchoServer's always-outstanding
  // async_read) -- with no pending async operation left, io_context::run() would otherwise return
  // on its own the moment the handshake handler finishes, ending thread_ and leaving any later
  // send() posted to a dead io_context to block forever. The work guard keeps run() alive
  // regardless of pending operations until stop() releases it.
  asio::executor_work_guard<asio::io_context::executor_type> work_guard_;
  std::thread thread_;
  std::atomic<bool> stopped_{ false };
  std::atomic<bool> connected_{ false };
  std::shared_ptr<websocket::stream<tcp::socket>> current_ws_;
};

nlohmann::json taskState(const std::string& id, const std::string& category, const std::string& status)
{
  nlohmann::json state;
  state["booking"]["id"] = id;
  state["category"] = category;
  state["status"] = status;
  return state;
}

}  // namespace

class MeasurementMissionOpenRmfTest : public ::testing::Test
{
protected:
  MeasurementMissionOpenRmfTest()
  {
    SetUp();
  }

  void SetUp() override
  {
    server_ = std::make_unique<PushServer>();
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "mission" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/mission", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementMissionOpenRmfTest::dataCallback, this, std::placeholders::_1));
  }

  void TearDown() override
  {
    stopCollection();
    server_->stop();
  }

  void stopCollection()
  {
    if (stopped_)
    {
      return;
    }
    stopped_ = true;
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("mission.plugin", std::string("dc_measurements/MissionOpenRmf"));
    ms_node_->declare_parameter("mission.group_key", std::string("mission"));
    ms_node_->declare_parameter("mission.topic_output", std::string("/dc/measurement/mission"));
    ms_node_->declare_parameter("mission.websocket_url",
                                std::string("ws://127.0.0.1:") + std::to_string(server_->port()) + "/");
    ms_node_->declare_parameter("mission.polling_interval", 20);
    ms_node_->declare_parameter("mission.init_collect", false);
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
    ASSERT_TRUE(server_->waitForConnection(std::chrono::seconds(10))) << "Plugin never connected to PushServer";
  }

  void dataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    records_.push_back(nlohmann::json::parse(data_str));
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  nlohmann::json waitForRecord(const std::function<bool(const nlohmann::json&)>& predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline)
    {
      for (const auto& record : records_)
      {
        if (predicate(record))
        {
          return record;
        }
      }
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ADD_FAILURE() << "No matching Record within the timeout";
    return nlohmann::json{};
  }

  static std::function<bool(const nlohmann::json&)> isEvent(const std::string& event)
  {
    return [event](const nlohmann::json& record) { return record.value("event", "") == event; };
  }

  static void expectValidatesAgainstSchema(const nlohmann::json& record)
  {
    const std::string path = ament_index_cpp::get_package_share_directory("dc_measurements") +
                             "/plugins/measurements/json/mission_open_rmf.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }

  std::unique_ptr<PushServer> server_;
  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  std::vector<nlohmann::json> records_;

  bool stopped_{ false };
};

TEST_F(MeasurementMissionOpenRmfTest, UnderwayEmitsAMissionStartRecord)
{
  declareCommonParameters();
  startLifecycleNode();

  server_->send(taskState("delivery.dispatch-1", "delivery", "underway"));
  const auto start = waitForRecord(isEvent("mission_start"));

  EXPECT_EQ(start["mission_id"], "delivery.dispatch-1");
  EXPECT_EQ(start["mission_type"], "delivery");
  EXPECT_EQ(start["sequence"], 1);
  expectValidatesAgainstSchema(start);
}

TEST_F(MeasurementMissionOpenRmfTest, QueuedAndStandbyEmitNoRecord)
{
  declareCommonParameters();
  startLifecycleNode();

  server_->send(taskState("delivery.dispatch-1", "delivery", "queued"));
  server_->send(taskState("delivery.dispatch-1", "delivery", "standby"));
  spinFor(std::chrono::milliseconds(200));

  EXPECT_TRUE(records_.empty());
}

TEST_F(MeasurementMissionOpenRmfTest, CompletedEmitsAMissionEndRecordWithSucceededOutcome)
{
  declareCommonParameters();
  startLifecycleNode();

  server_->send(taskState("delivery.dispatch-1", "delivery", "underway"));
  const auto start = waitForRecord(isEvent("mission_start"));

  server_->send(taskState("delivery.dispatch-1", "delivery", "completed"));
  const auto end = waitForRecord(isEvent("mission_end"));

  EXPECT_EQ(end["mission_id"], start["mission_id"]);
  EXPECT_EQ(end["outcome"], "succeeded");
  EXPECT_GE(end["duration_sec"].get<double>(), 0.0);
  EXPECT_FALSE(end.contains("reason"));
  EXPECT_FALSE(end.contains("error_code"));
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionOpenRmfTest, FailedCarriesReasonAndErrorCodeFromDispatchErrors)
{
  declareCommonParameters();
  startLifecycleNode();

  server_->send(taskState("delivery.dispatch-1", "delivery", "underway"));
  waitForRecord(isEvent("mission_start"));

  auto failed = taskState("delivery.dispatch-1", "delivery", "failed");
  nlohmann::json error;
  error["code"] = 12;
  error["category"] = "dispenser_unavailable";
  error["detail"] = "dispenser_1 did not respond";
  failed["dispatch"]["errors"] = nlohmann::json::array({ error });
  server_->send(failed);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "failed");
  EXPECT_EQ(end["reason"], "dispenser_unavailable: dispenser_1 did not respond");
  EXPECT_EQ(end["error_code"], 12);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionOpenRmfTest, CanceledCarriesReasonFromCancellationLabels)
{
  declareCommonParameters();
  startLifecycleNode();

  server_->send(taskState("patrol.dispatch-9", "patrol", "underway"));
  waitForRecord(isEvent("mission_start"));

  auto canceled = taskState("patrol.dispatch-9", "patrol", "canceled");
  canceled["cancellation"]["unix_millis_request_time"] = 0;
  canceled["cancellation"]["labels"] = nlohmann::json::array({ "operator", "dashboard" });
  server_->send(canceled);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "cancelled");
  EXPECT_EQ(end["reason"], "operator; dashboard");
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionOpenRmfTest, KilledCarriesReasonFromKilledLabels)
{
  declareCommonParameters();
  startLifecycleNode();

  server_->send(taskState("patrol.dispatch-9", "patrol", "underway"));
  waitForRecord(isEvent("mission_start"));

  auto killed = taskState("patrol.dispatch-9", "patrol", "killed");
  killed["killed"]["unix_millis_request_time"] = 0;
  killed["killed"]["labels"] = nlohmann::json::array({ "safety_stop" });
  server_->send(killed);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "aborted");
  EXPECT_EQ(end["reason"], "safety_stop");
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionOpenRmfTest, SkippedMapsToCancelledOutcome)
{
  declareCommonParameters();
  startLifecycleNode();

  server_->send(taskState("patrol.dispatch-9", "patrol", "underway"));
  waitForRecord(isEvent("mission_start"));

  server_->send(taskState("patrol.dispatch-9", "patrol", "skipped"));
  const auto end = waitForRecord(isEvent("mission_end"));

  EXPECT_EQ(end["outcome"], "cancelled");
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionOpenRmfTest, BlockedDoesNotEndTheMission)
{
  declareCommonParameters();
  startLifecycleNode();

  server_->send(taskState("delivery.dispatch-1", "delivery", "underway"));
  waitForRecord(isEvent("mission_start"));

  server_->send(taskState("delivery.dispatch-1", "delivery", "blocked"));
  spinFor(std::chrono::milliseconds(200));
  EXPECT_EQ(records_.size(), 1u) << "blocked must not produce a mission_end Record";

  server_->send(taskState("delivery.dispatch-1", "delivery", "completed"));
  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "succeeded");
}

TEST_F(MeasurementMissionOpenRmfTest, TerminalWithoutPriorActiveProducesNoRecords)
{
  declareCommonParameters();
  startLifecycleNode();

  server_->send(taskState("delivery.dispatch-1", "delivery", "completed"));
  spinFor(std::chrono::milliseconds(200));

  EXPECT_TRUE(records_.empty());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return all_successful;
}
