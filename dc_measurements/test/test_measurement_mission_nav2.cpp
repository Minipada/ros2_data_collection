// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <nlohmann/json-schema.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

// Stands in for whatever nav2's bt_navigator and its commander (a fleet orchestrator, an operator
// command, ...) really are: a real rclcpp_action::Server the plugin under test watches, and a real
// rclcpp_action::Client that drives it, completely independent of dc_measurements::MissionNav2 --
// exactly the passive-watcher relationship the plugin has to the real thing.
class MeasurementMissionNav2Test : public ::testing::Test
{
protected:
  MeasurementMissionNav2Test()
  {
    SetUp();
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "mission" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/mission", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementMissionNav2Test::dataCallback, this, std::placeholders::_1));

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

  void TearDown() override
  {
    stopCollection();
  }

  // Idempotent: one test stops collection mid-mission on purpose.
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
    ms_node_->declare_parameter("mission.plugin", std::string("dc_measurements/MissionNav2"));
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
  }

  void dataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    records_.push_back(nlohmann::json::parse(data_str));
  }

  void spinAll()
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(commander_node_);
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      spinAll();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Sends a goal through the fake commander and spins until the fake server's handle_accepted has
  // fired, so the test controls exactly when (and how) the goal completes.
  void sendGoalAndWaitForAcceptance()
  {
    ASSERT_TRUE(commander_client_->wait_for_action_server(std::chrono::seconds(5)));
    NavigateToPose::Goal goal;
    client_goal_handle_future_ = commander_client_->async_send_goal(goal);
    for (int i = 0; i < 400 && !goal_handle_; ++i)
    {
      spinAll();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(goal_handle_) << "Fake nav2 action server never reported the goal as accepted";
  }

  // Drives a real cancel request through the commander so the server-side goal handle legally
  // reaches CANCELING before the test completes it -- succeed()/abort() are reachable directly
  // from EXECUTING, but canceled() is only a legal transition out of CANCELING.
  void requestCancelAndWaitForCanceling()
  {
    for (int i = 0;
         i < 400 && client_goal_handle_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready; ++i)
    {
      spinAll();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(client_goal_handle_future_.wait_for(std::chrono::seconds(0)), std::future_status::ready);
    commander_client_->async_cancel_goal(client_goal_handle_future_.get());
    for (int i = 0; i < 400 && !goal_handle_->is_canceling(); ++i)
    {
      spinAll();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(goal_handle_->is_canceling());
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
    for (int i = 0; i < 600; ++i)
    {
      spinAll();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      for (size_t r = first_new; r < records_.size(); ++r)
      {
        if (predicate(records_[r]))
        {
          return records_[r];
        }
      }
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
                             "/plugins/measurements/json/mission_nav2.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }

  static constexpr const char* kActionName = "/test/navigate_to_pose";
  static int instance_;

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;

  rclcpp::Node::SharedPtr server_node_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  std::shared_ptr<NavigateToPoseGoalHandle> goal_handle_;

  rclcpp::Node::SharedPtr commander_node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr commander_client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> client_goal_handle_future_;

  std::vector<nlohmann::json> records_;
  bool stopped_{ false };
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
  spinFor(std::chrono::milliseconds(100));

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
  spinFor(std::chrono::milliseconds(200));

  ASSERT_EQ(records_.size(), 1u) << "Shutdown must not invent a closing Record";
  EXPECT_EQ(records_.back()["event"], "mission_start");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return all_successful;
}
