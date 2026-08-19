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
#include "moveit_msgs/action/move_group.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using MoveGroup = moveit_msgs::action::MoveGroup;

// A minimal action-server stub standing in for MoveIt's move_group, matching #387's precedent for
// testing this repo's first action-client-backed Measurement: it accepts every goal it receives
// and executes it, but never completes one on its own -- the test drives completion explicitly
// with a chosen error_code/planning_time, exactly like flipping a fault or a battery pack's
// reported status.
class FakeMoveGroupServer
{
public:
  enum class Terminal
  {
    kSucceed,
    kAbort,
    kCancel,
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
      case Terminal::kCancel:
        handle->canceled(result);
        break;
    }
  }

private:
  rclcpp_action::Server<MoveGroup>::SharedPtr server_;
  mutable std::mutex mutex_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>> pending_handle_;
};

class MeasurementManipulationTest : public ::testing::Test
{
protected:
  MeasurementManipulationTest()
  {
    SetUp();
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "manipulation" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/manipulation", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementManipulationTest::dataCallback, this, std::placeholders::_1));

    helper_node_ = std::make_shared<rclcpp::Node>("manipulation_test_helper");
    fake_server_ = std::make_unique<FakeMoveGroupServer>(helper_node_, kActionName);
    goal_client_ = rclcpp_action::create_client<MoveGroup>(helper_node_, kActionName);
  }

  void TearDown() override
  {
    stopCollection();
  }

  // Idempotent: one test stops collection with a goal still open on purpose, and TearDown must
  // not then drive the lifecycle node through a transition it is no longer in a state for.
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
    ms_node_->declare_parameter("manipulation.plugin", std::string("dc_measurements/Manipulation"));
    ms_node_->declare_parameter("manipulation.group_key", std::string("manipulation"));
    ms_node_->declare_parameter("manipulation.topic_output", std::string("/dc/measurement/manipulation"));
    ms_node_->declare_parameter("manipulation.action_name", std::string(kActionName));
    ms_node_->declare_parameter("manipulation.group_name", std::string("arm"));
    ms_node_->declare_parameter("manipulation.polling_interval", 50);
    ms_node_->declare_parameter("manipulation.init_collect", false);
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

  void spinFor(std::chrono::milliseconds duration)
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      rclcpp::spin_some(helper_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Spins both nodes until `predicate` is true or the timeout elapses.
  void spinUntil(const std::function<bool()>& predicate, std::chrono::milliseconds timeout)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
      rclcpp::spin_some(helper_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void waitForActionServer()
  {
    spinUntil([this] { return goal_client_->action_server_is_ready(); }, std::chrono::milliseconds(2000));
    ASSERT_TRUE(goal_client_->action_server_is_ready()) << "Manipulation never subscribed to the status topic";
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
          if (!fake_server_->hasPendingGoal())
          {
            return false;
          }
          for (size_t r = first_new; r < records_.size(); ++r)
          {
            if (records_[r].value("event", "") == "manipulation_start")
            {
              return true;
            }
          }
          return false;
        },
        std::chrono::milliseconds(4000));

    for (size_t r = first_new; r < records_.size(); ++r)
    {
      if (records_[r].value("event", "") == "manipulation_start")
      {
        return records_[r];
      }
    }
    ADD_FAILURE() << "No manipulation_start Record within the timeout";
    return nlohmann::json{};
  }

  nlohmann::json completeGoalAndWaitForEnd(FakeMoveGroupServer::Terminal terminal, int32_t error_code,
                                           double planning_time)
  {
    const size_t first_new = records_.size();
    fake_server_->completeGoal(terminal, error_code, planning_time);

    spinUntil(
        [this, first_new] {
          for (size_t r = first_new; r < records_.size(); ++r)
          {
            if (records_[r].value("event", "") == "manipulation_end")
            {
              return true;
            }
          }
          return false;
        },
        std::chrono::milliseconds(4000));

    for (size_t r = first_new; r < records_.size(); ++r)
    {
      if (records_[r].value("event", "") == "manipulation_end")
      {
        return records_[r];
      }
    }
    ADD_FAILURE() << "No manipulation_end Record within the timeout";
    return nlohmann::json{};
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

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Node::SharedPtr helper_node_;
  std::unique_ptr<FakeMoveGroupServer> fake_server_;
  rclcpp_action::Client<MoveGroup>::SharedPtr goal_client_;
  std::vector<nlohmann::json> records_;

  bool stopped_{ false };
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
  const auto end = completeGoalAndWaitForEnd(FakeMoveGroupServer::Terminal::kCancel, -7 /* PREEMPTED */, 0.0);

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

  spinFor(std::chrono::milliseconds(300));

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
  spinFor(std::chrono::milliseconds(200));

  ASSERT_EQ(records_.size(), 1u) << "Shutdown must not invent a manipulation_end Record";
  EXPECT_EQ(records_.back()["event"], "manipulation_start");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
