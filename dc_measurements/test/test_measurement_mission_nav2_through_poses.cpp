// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json-schema.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/msg/waypoint_status.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{
constexpr const char* kActionName = "/test/navigate_through_poses";
}  // namespace

// A minimal rclcpp_action::Server standing in for nav2's real bt_navigator, driven by a real
// rclcpp_action::Client the test owns -- the Measurement under test never sends a goal itself, it
// only watches this server's status/feedback/get_result endpoints (#388's passive-observer
// design), so something else has to originate the goal the way the real BT would.
class MeasurementMissionNav2ThroughPosesTest : public ::testing::Test
{
protected:
  using ActionT = nav2_msgs::action::NavigateThroughPoses;
  using ServerGoalHandleT = rclcpp_action::ServerGoalHandle<ActionT>;
  using ClientGoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;

  MeasurementMissionNav2ThroughPosesTest()
  {
    SetUp();
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "mission" });
    server_node_ = std::make_shared<rclcpp::Node>("fake_nav2_through_poses_server");

    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/mission", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementMissionNav2ThroughPosesTest::dataCallback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<ActionT>(
        server_node_, kActionName,
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const ActionT::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<ServerGoalHandleT>&) { return rclcpp_action::CancelResponse::ACCEPT; },
        std::bind(&MeasurementMissionNav2ThroughPosesTest::handleAccepted, this, std::placeholders::_1));
    action_client_ = rclcpp_action::create_client<ActionT>(server_node_, kActionName);
  }

  void TearDown() override
  {
    stopCollection();
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
    ms_node_->declare_parameter("mission.plugin", std::string("dc_measurements/MissionNav2ThroughPoses"));
    ms_node_->declare_parameter("mission.group_key", std::string("mission"));
    ms_node_->declare_parameter("mission.topic_output", std::string("/dc/measurement/mission"));
    ms_node_->declare_parameter("mission.action_name", std::string(kActionName));
    ms_node_->declare_parameter("mission.polling_interval", 50);
    ms_node_->declare_parameter("mission.init_collect", false);
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
    waitForStatusSubscriber();
  }

  void dataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    records_.push_back(nlohmann::json::parse(data_str));
  }

  void handleAccepted(const std::shared_ptr<ServerGoalHandleT>& goal_handle)
  {
    {
      const std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_ = goal_handle;
    }
    // Standing in for a real action server that starts working immediately on acceptance -- moves
    // the goal from ACCEPTED straight to EXECUTING so succeed()/abort() below are valid calls, and
    // so a client-initiated cancel lands in CANCELING rather than being rejected outright.
    goal_handle->execute();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      rclcpp::spin_some(server_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void waitForStatusSubscriber()
  {
    const std::string topic = std::string(kActionName) + "/_action/status";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (server_node_->count_subscribers(topic) == 0 && std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      rclcpp::spin_some(server_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  std::shared_ptr<ClientGoalHandleT> sendGoal()
  {
    auto goal_msg = ActionT::Goal();
    auto goal_handle_future = action_client_->async_send_goal(goal_msg);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (goal_handle_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
           std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(server_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    auto goal_handle = goal_handle_future.get();
    EXPECT_NE(goal_handle, nullptr) << "Goal was not accepted within the timeout";
    return goal_handle;
  }

  std::shared_ptr<ServerGoalHandleT> waitForActiveGoal()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline)
    {
      {
        const std::lock_guard<std::mutex> lock(goal_mutex_);
        if (active_goal_)
        {
          return active_goal_;
        }
      }
      rclcpp::spin_some(server_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ADD_FAILURE() << "No goal was accepted within the timeout";
    return nullptr;
  }

  void publishFeedback(int recoveries)
  {
    auto feedback = std::make_shared<ActionT::Feedback>();
    feedback->number_of_recoveries = recoveries;
    waitForActiveGoal()->publish_feedback(feedback);
  }

  static ActionT::Result::SharedPtr makeResult(uint16_t error_code, const std::string& error_msg,
                                               std::vector<nav2_msgs::msg::WaypointStatus> waypoint_statuses = {})
  {
    auto result = std::make_shared<ActionT::Result>();
    result->error_code = error_code;
    result->error_msg = error_msg;
    result->waypoint_statuses = std::move(waypoint_statuses);
    return result;
  }

  void succeedActiveGoal(uint16_t error_code = 0, const std::string& error_msg = "",
                         std::vector<nav2_msgs::msg::WaypointStatus> waypoint_statuses = {})
  {
    waitForActiveGoal()->succeed(makeResult(error_code, error_msg, std::move(waypoint_statuses)));
  }

  void abortActiveGoal(uint16_t error_code, const std::string& error_msg)
  {
    waitForActiveGoal()->abort(makeResult(error_code, error_msg));
  }

  void cancelActiveGoal(const std::shared_ptr<ClientGoalHandleT>& client_goal_handle)
  {
    auto cancel_future = action_client_->async_cancel_goal(client_goal_handle);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (cancel_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
           std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(server_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    waitForActiveGoal()->canceled(makeResult(0, ""));
  }

  // Spins both nodes until a Record matching `predicate` shows up, so the async accept/execute/
  // get_result round trip doesn't make the test flaky.
  nlohmann::json waitForRecord(const std::function<bool(const nlohmann::json&)>& predicate)
  {
    const size_t first_new = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline)
    {
      for (size_t r = first_new; r < records_.size(); ++r)
      {
        if (predicate(records_[r]))
        {
          return records_[r];
        }
      }
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      rclcpp::spin_some(server_node_);
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
                             "/plugins/measurements/json/mission_nav2_through_poses.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Node::SharedPtr server_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  std::vector<nlohmann::json> records_;

  std::mutex goal_mutex_;
  std::shared_ptr<ServerGoalHandleT> active_goal_;

  bool stopped_{ false };
};

TEST_F(MeasurementMissionNav2ThroughPosesTest, GoalAcceptedEmitsAMissionStartRecord)
{
  declareCommonParameters();
  startLifecycleNode();

  sendGoal();
  const auto start = waitForRecord(isEvent("mission_start"));

  EXPECT_EQ(start["sequence"], 1);
  EXPECT_EQ(start["mission_type"], "navigate_through_poses");
  EXPECT_FALSE(start["mission_id"].get<std::string>().empty());
  expectValidatesAgainstSchema(start);
}

TEST_F(MeasurementMissionNav2ThroughPosesTest, GoalSucceedsEmitsAMissionEndRecordWithSucceededOutcome)
{
  declareCommonParameters();
  startLifecycleNode();

  sendGoal();
  const auto start = waitForRecord(isEvent("mission_start"));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  succeedActiveGoal();

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["mission_id"], start["mission_id"]);
  EXPECT_EQ(end["outcome"], "succeeded");
  EXPECT_EQ(end["mission_type"], "navigate_through_poses");
  EXPECT_GE(end["duration_sec"].get<double>(), 0.0);
  EXPECT_FALSE(end.contains("reason"));
  EXPECT_FALSE(end.contains("error_code"));
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2ThroughPosesTest, GoalAbortedCarriesReasonAndErrorCode)
{
  declareCommonParameters();
  startLifecycleNode();

  sendGoal();
  waitForRecord(isEvent("mission_start"));
  abortActiveGoal(9102, "tf timeout");

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "aborted");
  EXPECT_EQ(end["reason"], "tf timeout");
  EXPECT_EQ(end["error_code"], 9102);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2ThroughPosesTest, GoalCanceledIsOutcomeCancelled)
{
  declareCommonParameters();
  startLifecycleNode();

  auto client_goal_handle = sendGoal();
  waitForRecord(isEvent("mission_start"));
  cancelActiveGoal(client_goal_handle);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "cancelled");
  EXPECT_FALSE(end.contains("reason"));
  EXPECT_FALSE(end.contains("error_code"));
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2ThroughPosesTest, NonZeroErrorCodeOnASucceededStatusIsOutcomeFailed)
{
  declareCommonParameters();
  startLifecycleNode();

  sendGoal();
  waitForRecord(isEvent("mission_start"));
  succeedActiveGoal(9101, "failed to load behavior tree");

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "failed");
  EXPECT_EQ(end["reason"], "failed to load behavior tree");
  EXPECT_EQ(end["error_code"], 9101);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2ThroughPosesTest, RecoveriesFromFeedbackAreCarriedOntoMissionEnd)
{
  declareCommonParameters();
  startLifecycleNode();

  sendGoal();
  waitForRecord(isEvent("mission_start"));
  publishFeedback(3);
  spinFor(std::chrono::milliseconds(100));
  succeedActiveGoal();

  const auto end = waitForRecord(isEvent("mission_end"));
  ASSERT_TRUE(end.contains("recoveries"));
  EXPECT_EQ(end["recoveries"], 3);
}

TEST_F(MeasurementMissionNav2ThroughPosesTest, WaypointStatusesAreAggregatedOntoMissionEnd)
{
  declareCommonParameters();
  startLifecycleNode();

  sendGoal();
  waitForRecord(isEvent("mission_start"));

  std::vector<nav2_msgs::msg::WaypointStatus> waypoints(3);
  waypoints[0].waypoint_status = nav2_msgs::msg::WaypointStatus::COMPLETED;
  waypoints[1].waypoint_status = nav2_msgs::msg::WaypointStatus::COMPLETED;
  waypoints[2].waypoint_status = nav2_msgs::msg::WaypointStatus::FAILED;
  succeedActiveGoal(0, "", waypoints);

  const auto end = waitForRecord(isEvent("mission_end"));
  ASSERT_TRUE(end.contains("waypoint_statuses"));
  EXPECT_EQ(end["waypoint_statuses"]["total"], 3);
  EXPECT_EQ(end["waypoint_statuses"]["completed"], 2);
  EXPECT_EQ(end["waypoint_statuses"]["failed"], 1);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2ThroughPosesTest, SequenceIsMonotonicAcrossSeveralMissions)
{
  declareCommonParameters();
  startLifecycleNode();

  sendGoal();
  const auto start1 = waitForRecord(isEvent("mission_start"));
  succeedActiveGoal();
  const auto end1 = waitForRecord([&](const nlohmann::json& r) {
    return r.value("event", "") == "mission_end" && r["mission_id"] == start1["mission_id"];
  });

  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_.reset();
  }

  sendGoal();
  const auto start2 = waitForRecord([&](const nlohmann::json& r) {
    return r.value("event", "") == "mission_start" && r["mission_id"] != start1["mission_id"];
  });
  abortActiveGoal(9103, "timeout");
  const auto end2 = waitForRecord([&](const nlohmann::json& r) {
    return r.value("event", "") == "mission_end" && r["mission_id"] == start2["mission_id"];
  });

  EXPECT_LT(start1["sequence"].get<uint64_t>(), end1["sequence"].get<uint64_t>());
  EXPECT_LT(end1["sequence"].get<uint64_t>(), start2["sequence"].get<uint64_t>());
  EXPECT_LT(start2["sequence"].get<uint64_t>(), end2["sequence"].get<uint64_t>());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return all_successful;
}
