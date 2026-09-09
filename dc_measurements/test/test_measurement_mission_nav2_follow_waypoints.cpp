// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <fstream>
#include <functional>
#include <memory>
#include <nlohmann/json-schema.hpp>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dc_measurements/measurement.hpp"
#include "measurement_test_bench.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/msg/missed_waypoint.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using FollowWaypointsGoalHandle = rclcpp_action::ServerGoalHandle<FollowWaypoints>;

// Stands in for whatever nav2's WaypointFollower action server and its commander (nav2_simple_commander,
// a WMS integration, ...) really are: a real rclcpp_action::Server the plugin under test watches, and a
// real rclcpp_action::Client that drives it, completely independent of dc_measurements::MissionNav2FollowWaypoints
// -- exactly the passive-watcher relationship the plugin has to the real thing.
class MeasurementMissionNav2FollowWaypointsTest : public MeasurementBench
{
protected:
  MeasurementMissionNav2FollowWaypointsTest() : MeasurementBench("mission")
  {
    server_node_ = std::make_shared<rclcpp::Node>("fake_nav2_" + std::to_string(instance_++));
    action_server_ = rclcpp_action::create_server<FollowWaypoints>(
        server_node_, kActionName,
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowWaypoints::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<FollowWaypointsGoalHandle>&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const std::shared_ptr<FollowWaypointsGoalHandle>& goal_handle) { goal_handle_ = goal_handle; });

    commander_node_ = std::make_shared<rclcpp::Node>("fake_commander_" + std::to_string(instance_++));
    commander_client_ = rclcpp_action::create_client<FollowWaypoints>(commander_node_, kActionName);
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("mission.plugin", std::string("dc_measurements/MissionNav2FollowWaypoints"));
    ms_node_->declare_parameter("mission.group_key", std::string("mission"));
    ms_node_->declare_parameter("mission.topic_output", std::string("/dc/measurement/mission"));
    ms_node_->declare_parameter("mission.action_name", std::string(kActionName));
    ms_node_->declare_parameter("mission.polling_interval", 50);
    ms_node_->declare_parameter("mission.init_collect", false);
  }

  // The fake nav2 server and commander live on their own nodes, which the bench's waits spin
  // alongside the MeasurementServer.
  void spinExtra() override
  {
    rclcpp::spin_some(server_node_->get_node_base_interface());
    rclcpp::spin_some(commander_node_->get_node_base_interface());
  }

  // Sends a goal through the fake commander and spins until the fake server's handle_accepted has
  // fired, so the test controls exactly when (and how) the goal completes.
  void sendGoalAndWaitForAcceptance()
  {
    ASSERT_TRUE(commander_client_->wait_for_action_server(std::chrono::seconds(5)));
    FollowWaypoints::Goal goal;
    client_goal_handle_future_ = commander_client_->async_send_goal(goal);
    ASSERT_TRUE(spinUntil([this] { return goal_handle_ != nullptr; }, 4000))
        << "Fake nav2 action server never reported the goal as accepted";
  }

  // Drives a real cancel request through the commander so the server-side goal handle legally
  // reaches CANCELING before the test completes it -- succeed()/abort() are reachable directly
  // from EXECUTING, but canceled() is only a legal transition out of CANCELING.
  void requestCancelAndWaitForCanceling()
  {
    ASSERT_TRUE(spinUntil(
        [this] { return client_goal_handle_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
        4000))
        << "the goal response never arrived";
    commander_client_->async_cancel_goal(client_goal_handle_future_.get());
    ASSERT_TRUE(spinUntil([this] { return goal_handle_->is_canceling(); }, 4000))
        << "the goal handle never reached CANCELING";
  }

  nlohmann::json waitForRecord(const std::function<bool(const nlohmann::json&)>& predicate)
  {
    const size_t first_new = records_.size();
    nlohmann::json matched;
    if (!spinUntil(
            [&] {
              for (size_t r = first_new; r < records_.size(); ++r)
              {
                if (predicate(records_[r]))
                {
                  matched = records_[r];
                  return true;
                }
              }
              return false;
            },
            6000))
    {
      ADD_FAILURE() << "No matching Record within the timeout";
      return nlohmann::json{};
    }
    return matched;
  }

  static std::function<bool(const nlohmann::json&)> isEvent(const std::string& event)
  {
    return [event](const nlohmann::json& record) { return record.value("event", "") == event; };
  }

  static void expectValidatesAgainstSchema(const nlohmann::json& record)
  {
    const std::string schema_dir =
        ament_index_cpp::get_package_share_directory("dc_measurements") + "/plugins/measurements/json";
    const std::string path = schema_dir + "/mission_nav2_follow_waypoints.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    // mission_nav2_follow_waypoints.json's allOf $refs mission_base.json (#305/ADR-0010's shared
    // property definitions), so this independent re-validation needs the same loader
    // dc_measurements::Measurement::validateSchema() uses in production.
    nlohmann::json_schema::json_validator validator(dc_measurements::makeSchemaFileLoader(schema_dir));
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }

  static constexpr const char* kActionName = "/test/follow_waypoints";
  static int instance_;

  rclcpp::Node::SharedPtr server_node_;
  rclcpp_action::Server<FollowWaypoints>::SharedPtr action_server_;
  std::shared_ptr<FollowWaypointsGoalHandle> goal_handle_;

  rclcpp::Node::SharedPtr commander_node_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr commander_client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr> client_goal_handle_future_;
};

int MeasurementMissionNav2FollowWaypointsTest::instance_ = 0;

TEST_F(MeasurementMissionNav2FollowWaypointsTest, AcceptedGoalProducesAMissionStartRecord)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();

  const auto start = waitForRecord(isEvent("mission_start"));
  EXPECT_EQ(start["mission_type"], "follow_waypoints");
  EXPECT_GT(start["sequence"].get<uint64_t>(), 0u);
  EXPECT_FALSE(start["mission_id"].get<std::string>().empty());
  expectValidatesAgainstSchema(start);
}

TEST_F(MeasurementMissionNav2FollowWaypointsTest, SucceededGoalProducesAMissionEndWithNoReason)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();
  const auto start = waitForRecord(isEvent("mission_start"));

  auto result = std::make_shared<FollowWaypoints::Result>();
  result->error_code = 0;
  goal_handle_->succeed(result);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["mission_id"], start["mission_id"]);
  EXPECT_EQ(end["mission_type"], "follow_waypoints");
  EXPECT_EQ(end["sequence"].get<uint64_t>(), start["sequence"].get<uint64_t>() + 1);
  EXPECT_EQ(end["outcome"], "succeeded");
  EXPECT_GE(end["duration_sec"].get<double>(), 0.0);
  EXPECT_FALSE(end.contains("reason"));
  EXPECT_FALSE(end.contains("error_code"));
  EXPECT_TRUE(end["missed_waypoints"].empty());
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2FollowWaypointsTest, SucceededStatusWithNonZeroErrorCodeIsReportedAsFailed)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();
  waitForRecord(isEvent("mission_start"));

  auto result = std::make_shared<FollowWaypoints::Result>();
  result->error_code = FollowWaypoints::Result::TASK_EXECUTOR_FAILED;
  result->error_msg = "task executor failed";
  nav2_msgs::msg::MissedWaypoint missed;
  missed.index = 2;
  missed.error_code = FollowWaypoints::Result::TASK_EXECUTOR_FAILED;
  result->missed_waypoints.push_back(missed);
  goal_handle_->succeed(result);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "failed");
  EXPECT_EQ(end["reason"], "task executor failed");
  EXPECT_EQ(end["error_code"].get<int>(), FollowWaypoints::Result::TASK_EXECUTOR_FAILED);
  ASSERT_EQ(end["missed_waypoints"].size(), 1u);
  EXPECT_EQ(end["missed_waypoints"][0]["index"].get<int>(), 2);
  EXPECT_EQ(end["missed_waypoints"][0]["error_code"].get<int>(), FollowWaypoints::Result::TASK_EXECUTOR_FAILED);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2FollowWaypointsTest, CanceledGoalIsReportedAsCancelled)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();
  waitForRecord(isEvent("mission_start"));
  requestCancelAndWaitForCanceling();

  auto result = std::make_shared<FollowWaypoints::Result>();
  goal_handle_->canceled(result);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "cancelled");
  EXPECT_FALSE(end.contains("reason"));
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2FollowWaypointsTest, AbortedGoalCarriesReasonAndErrorCode)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();
  waitForRecord(isEvent("mission_start"));

  auto result = std::make_shared<FollowWaypoints::Result>();
  result->error_code = FollowWaypoints::Result::UNKNOWN;
  result->error_msg = "unknown failure";
  goal_handle_->abort(result);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "aborted");
  EXPECT_EQ(end["reason"], "unknown failure");
  EXPECT_EQ(end["error_code"].get<int>(), FollowWaypoints::Result::UNKNOWN);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2FollowWaypointsTest, MissionOpenAtShutdownGetsNoClosingRecord)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();
  waitForRecord(isEvent("mission_start"));

  stopCollection();
  spinFor(200);

  ASSERT_EQ(records_.size(), 1u) << "Shutdown must not invent a closing Record";
  EXPECT_EQ(records_.back()["event"], "mission_start");
}

DC_MEASUREMENT_TEST_MAIN()
