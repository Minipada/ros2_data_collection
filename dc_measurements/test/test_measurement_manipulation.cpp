// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <functional>
#include <memory>
#include <moveit_msgs/action/move_group.hpp>
#include <mutex>
#include <nlohmann/json-schema.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>
#include <vector>

#include "measurement_test_bench.hpp"

using MoveGroup = moveit_msgs::action::MoveGroup;

class FakeMoveGroupServer
{
public:
  enum class Terminal
  {
    kSucceed,
    kAbort,
  };

  FakeMoveGroupServer(const rclcpp::Node::SharedPtr& node, const std::string& action_name)
  {
    server_ = rclcpp_action::create_server<MoveGroup>(
        node, action_name,
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveGroup::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>> goal_handle) {
          const std::lock_guard<std::mutex> lock(mutex_);
          pending_handle_ = goal_handle;
        });
  }

  bool hasPendingGoal() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return pending_handle_ != nullptr;
  }

  // Completes the most recently accepted goal. Any client that knows its goal_id -- this
  // Measurement included -- can retrieve the Result this produces via the action's get_result
  // service, not only the client that originally sent the goal.
  void completeGoal(Terminal terminal, int32_t error_code, double planning_time)
  {
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>> handle;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      handle = pending_handle_;
      pending_handle_.reset();
    }
    ASSERT_TRUE(handle != nullptr) << "completeGoal() called with no pending goal";

    auto result = std::make_shared<MoveGroup::Result>();
    result->error_code.val = error_code;
    result->planning_time = planning_time;
    switch (terminal)
    {
      case Terminal::kSucceed:
        handle->succeed(result);
        break;
      case Terminal::kAbort:
        handle->abort(result);
        break;
    }
  }

private:
  rclcpp_action::Server<MoveGroup>::SharedPtr server_;
  mutable std::mutex mutex_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>> pending_handle_;
};

class MeasurementManipulationTest : public MeasurementBench
{
protected:
  MeasurementManipulationTest() : MeasurementBench("manipulation")
  {
    helper_node_ = std::make_shared<rclcpp::Node>("manipulation_test_helper");
    fake_server_ = std::make_unique<FakeMoveGroupServer>(helper_node_, kActionName);
    goal_client_ = rclcpp_action::create_client<MoveGroup>(helper_node_, kActionName);
  }

  void declareCommonParameters()
  {
    ms_node_->declare_parameter("manipulation.plugin", std::string("dc_measurements/Manipulation"));
    ms_node_->declare_parameter("manipulation.group_key", std::string("manipulation"));
    ms_node_->declare_parameter("manipulation.topic_output", std::string("/dc/measurement/manipulation"));
    ms_node_->declare_parameter("manipulation.action_name", std::string(kActionName));
    ms_node_->declare_parameter("manipulation.group_name", std::string("arm"));
    ms_node_->declare_parameter("manipulation.polling_interval", 50);
    ms_node_->declare_parameter("manipulation.init_collect", false);
  }

  // The fake action server lives on its own node, which the bench's waits spin alongside the
  // MeasurementServer.
  void spinExtra() override
  {
    rclcpp::spin_some(helper_node_->get_node_base_interface());
  }

  void waitForActionServer()
  {
    ASSERT_TRUE(spinUntil([this] { return goal_client_->action_server_is_ready(); }, 2000))
        << "Manipulation never subscribed to the status topic";
  }

  // Sends a default-constructed goal (this Measurement never reads the goal's own fields -- its
  // group_name is configured, not observed) and waits until both the fake server has it and the
  // Manipulation Measurement has reported it.
  nlohmann::json sendGoalAndWaitForStart()
  {
    const size_t first_new = records_.size();
    MoveGroup::Goal goal;
    goal_client_->async_send_goal(goal);

    spinUntil(
        [this, first_new] {
          return fake_server_->hasPendingGoal() && findRecord(first_new, "manipulation_start") != nullptr;
        },
        4000);

    auto record = findRecord(first_new, "manipulation_start");
    if (record == nullptr)
    {
      ADD_FAILURE() << "No manipulation_start Record within the timeout";
      return nlohmann::json{};
    }
    return *record;
  }

  nlohmann::json completeGoalAndWaitForEnd(FakeMoveGroupServer::Terminal terminal, int32_t error_code,
                                           double planning_time)
  {
    const size_t first_new = records_.size();
    fake_server_->completeGoal(terminal, error_code, planning_time);

    spinUntil([this, first_new] { return findRecord(first_new, "manipulation_end") != nullptr; }, 4000);

    auto record = findRecord(first_new, "manipulation_end");
    if (record == nullptr)
    {
      ADD_FAILURE() << "No manipulation_end Record within the timeout";
      return nlohmann::json{};
    }
    return *record;
  }

  // The first Record from `first_new` on whose "event" is `event`, or nullptr.
  const nlohmann::json* findRecord(size_t first_new, const char* event)
  {
    for (size_t r = first_new; r < records_.size(); ++r)
    {
      if (records_[r].value("event", "") == event)
      {
        return &records_[r];
      }
    }
    return nullptr;
  }

  static void expectValidatesAgainstSchema(const nlohmann::json& record)
  {
    const std::string path = ament_index_cpp::get_package_share_directory("dc_measurements") +
                             "/plugins/measurements/json/manipulation.json";
    std::ifstream schema_file(path);
    ASSERT_TRUE(schema_file.good()) << "Schema not installed at " << path;
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(nlohmann::json::parse(schema_file));
    EXPECT_NO_THROW(validator.validate(record)) << record.dump();
  }

  static constexpr const char* kActionName = "/test/move_action";

  rclcpp::Node::SharedPtr helper_node_;
  std::unique_ptr<FakeMoveGroupServer> fake_server_;
  rclcpp_action::Client<MoveGroup>::SharedPtr goal_client_;
};

TEST_F(MeasurementManipulationTest, GoalAcceptedProducesAManipulationStartRecord)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForActionServer();

  const auto start = sendGoalAndWaitForStart();

  EXPECT_EQ(start["event"], "manipulation_start");
  EXPECT_EQ(start["group_name"], "arm");
  EXPECT_FALSE(start["goal_id"].get<std::string>().empty());
  EXPECT_GE(start["sequence"].get<uint64_t>(), 1u);
  expectValidatesAgainstSchema(start);
}

TEST_F(MeasurementManipulationTest, SuccessfulGoalProducesASucceededEndRecord)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForActionServer();

  const auto start = sendGoalAndWaitForStart();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));  // A measurable duration_sec.
  const auto end = completeGoalAndWaitForEnd(FakeMoveGroupServer::Terminal::kSucceed, 1 /* SUCCESS */, 1.25);

  EXPECT_EQ(end["event"], "manipulation_end");
  EXPECT_EQ(end["goal_id"], start["goal_id"]);
  EXPECT_EQ(end["group_name"], "arm");
  EXPECT_EQ(end["outcome"], "succeeded");
  EXPECT_EQ(end["error_code"].get<int>(), 1);
  EXPECT_NEAR(end["planning_time"].get<double>(), 1.25, 1e-6);
  EXPECT_GT(end["duration_sec"].get<double>(), 0.0);
  EXPECT_EQ(end["sequence"].get<uint64_t>(), start["sequence"].get<uint64_t>() + 1);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementManipulationTest, PreemptedGoalMapsToCancelledOutcome)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForActionServer();

  sendGoalAndWaitForStart();
  // Real MoveGroup reports a preempted goal as aborted with error_code PREEMPTED, not as
  // canceled -- rcl_action's own goal-handle state machine only allows the CANCELED terminal
  // state to be reached from CANCELING (i.e. after an actual accepted cancel request), never
  // directly from EXECUTING. Manipulation's outcome mapping is driven by error_code alone (see
  // Manipulation::onResult), so which terminal call produced it is deliberately irrelevant here.
  const auto end = completeGoalAndWaitForEnd(FakeMoveGroupServer::Terminal::kAbort, -7 /* PREEMPTED */, 0.0);

  EXPECT_EQ(end["outcome"], "cancelled");
  EXPECT_EQ(end["error_code"].get<int>(), -7);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementManipulationTest, NegativeErrorCodeOtherThanPreemptedMapsToFailedOutcome)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForActionServer();

  sendGoalAndWaitForStart();
  const auto end = completeGoalAndWaitForEnd(FakeMoveGroupServer::Terminal::kAbort, -1 /* PLANNING_FAILED */, 0.4);

  EXPECT_EQ(end["outcome"], "failed");
  EXPECT_EQ(end["error_code"].get<int>(), -1);
  expectValidatesAgainstSchema(end);
}

TEST_F(MeasurementManipulationTest, NoGoalSentProducesNoRecords)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForActionServer();

  spinFor(300);

  EXPECT_TRUE(records_.empty()) << records_.size() << " Record(s) published without any goal";
}

TEST_F(MeasurementManipulationTest, GoalOpenAtShutdownGetsNoEndRecord)
{
  declareCommonParameters();
  startLifecycleNode();
  waitForActionServer();

  sendGoalAndWaitForStart();

  // Collection stops with the goal still executing.
  stopCollection();
  spinFor(200);

  ASSERT_EQ(records_.size(), 1u) << "Shutdown must not invent a manipulation_end Record";
  EXPECT_EQ(records_.back()["event"], "manipulation_start");
}

DC_MEASUREMENT_TEST_MAIN()
