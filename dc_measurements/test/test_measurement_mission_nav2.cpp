// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <functional>
#include <memory>
#include <nlohmann/json-schema.hpp>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dc_measurements/measurement.hpp"
#include "measurement_test_bench.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

// Stands in for whatever nav2's bt_navigator and its commander (a fleet orchestrator, an operator
// command, ...) really are: a real rclcpp_action::Server the plugin under test watches, and a real
// rclcpp_action::Client that drives it, completely independent of dc_measurements::MissionNav2 --
// exactly the passive-watcher relationship the plugin has to the real thing.
class MeasurementMissionNav2Test : public MeasurementBench
{
protected:
  MeasurementMissionNav2Test() : MeasurementBench("mission")
  {
    server_node_ = std::make_shared<rclcpp::Node>("fake_nav2_" + std::to_string(instance_++));
    action_server_ = rclcpp_action::create_server<NavigateToPose>(
        server_node_, kActionName,
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const NavigateToPose::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<NavigateToPoseGoalHandle>&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const std::shared_ptr<NavigateToPoseGoalHandle>& goal_handle) { goal_handle_ = goal_handle; });

    commander_node_ = std::make_shared<rclcpp::Node>("fake_commander_" + std::to_string(instance_++));
    commander_client_ = rclcpp_action::create_client<NavigateToPose>(commander_node_, kActionName);
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("mission.plugin", std::string("dc_measurements/MissionNav2"));
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
    NavigateToPose::Goal goal;
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

  void publishFeedback(int16_t recoveries)
  {
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->number_of_recoveries = recoveries;
    goal_handle_->publish_feedback(feedback);
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
    const std::string path = schema_dir + "/mission_nav2.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    // mission_nav2.json's allOf $refs mission_base.json (#305/ADR-0010's shared property
    // definitions), so this independent re-validation needs the same loader
    // dc_measurements::Measurement::validateSchema() uses in production.
    nlohmann::json_schema::json_validator validator(dc_measurements::makeSchemaFileLoader(schema_dir));
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }

  static constexpr const char* kActionName = "/test/navigate_to_pose";
  static int instance_;

  rclcpp::Node::SharedPtr server_node_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  std::shared_ptr<NavigateToPoseGoalHandle> goal_handle_;

  rclcpp::Node::SharedPtr commander_node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr commander_client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> client_goal_handle_future_;
};

int MeasurementMissionNav2Test::instance_ = 0;

TEST_F(MeasurementMissionNav2Test, AcceptedGoalProducesAMissionStartRecord)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();

  const auto start = waitForRecord(isEvent("mission_start"));
  EXPECT_EQ(start["mission_type"], "navigate_to_pose");
  EXPECT_GT(start["sequence"].get<uint64_t>(), 0u);
  EXPECT_FALSE(start["mission_id"].get<std::string>().empty());
  expectValidatesAgainstSchema(start);
}

TEST_F(MeasurementMissionNav2Test, SucceededGoalProducesAMissionEndWithNoReason)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();
  const auto start = waitForRecord(isEvent("mission_start"));

  auto result = std::make_shared<NavigateToPose::Result>();
  result->error_code = 0;
  goal_handle_->succeed(result);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["mission_id"], start["mission_id"]);
  EXPECT_EQ(end["mission_type"], "navigate_to_pose");
  EXPECT_EQ(end["sequence"].get<uint64_t>(), start["sequence"].get<uint64_t>() + 1);
  EXPECT_EQ(end["outcome"], "succeeded");
  EXPECT_GE(end["duration_sec"].get<double>(), 0.0);
  EXPECT_FALSE(end.contains("reason"));
  EXPECT_FALSE(end.contains("error_code"));
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2Test, SucceededStatusWithNonZeroErrorCodeIsReportedAsFailed)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();
  waitForRecord(isEvent("mission_start"));

  auto result = std::make_shared<NavigateToPose::Result>();
  result->error_code = 9101;
  result->error_msg = "failed to load behavior tree";
  goal_handle_->succeed(result);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "failed");
  EXPECT_EQ(end["reason"], "failed to load behavior tree");
  EXPECT_EQ(end["error_code"].get<int>(), 9101);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2Test, CanceledGoalIsReportedAsCancelled)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();
  waitForRecord(isEvent("mission_start"));
  requestCancelAndWaitForCanceling();

  auto result = std::make_shared<NavigateToPose::Result>();
  goal_handle_->canceled(result);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "cancelled");
  EXPECT_FALSE(end.contains("reason"));
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2Test, AbortedGoalCarriesReasonAndErrorCode)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();
  waitForRecord(isEvent("mission_start"));

  auto result = std::make_shared<NavigateToPose::Result>();
  result->error_code = 9102;
  result->error_msg = "tf timeout";
  goal_handle_->abort(result);

  const auto end = waitForRecord(isEvent("mission_end"));
  EXPECT_EQ(end["outcome"], "aborted");
  EXPECT_EQ(end["reason"], "tf timeout");
  EXPECT_EQ(end["error_code"].get<int>(), 9102);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2Test, RecoveriesFromFeedbackAreCarriedOntoMissionEnd)
{
  declareCommonParameters();
  startLifecycleNode();
  sendGoalAndWaitForAcceptance();
  waitForRecord(isEvent("mission_start"));

  publishFeedback(3);
  spinFor(100);

  auto result = std::make_shared<NavigateToPose::Result>();
  goal_handle_->succeed(result);

  const auto end = waitForRecord(isEvent("mission_end"));
  ASSERT_TRUE(end.contains("recoveries"));
  EXPECT_EQ(end["recoveries"], 3);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementMissionNav2Test, MissionOpenAtShutdownGetsNoClosingRecord)
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
