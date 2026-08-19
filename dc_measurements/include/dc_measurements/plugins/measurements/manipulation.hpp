// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MANIPULATION_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MANIPULATION_HPP_

#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <utility>

#include "action_msgs/msg/goal_status_array.hpp"
#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_util/node_utils.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_measurements
{

/**
 * @class dc_measurements::Manipulation
 * @brief Watches MoveIt's MoveGroup action -- the flagship manipulation stack, same "native
 * action, cheap to adapt" profile as nav2 (#396) -- and emits a manipulation_start/
 * manipulation_end Record pair per goal.
 *
 * A passive observer, not the client that sends the goal: it reads the action's status topic
 * (`<action_name>/_action/status`, published for every goal regardless of which client sent it)
 * to see a goal appear and reach a terminal state, then calls the action's get_result service
 * (`<action_name>/_action/get_result`) -- a plain service any client may call given the goal's
 * UUID, not only the one that sent it -- to read MoveGroup::Result's error_code and planning_time.
 * `group_name` is this instance's own configuration rather than read off the goal: MoveGroup's
 * Goal (the only place a group name appears) is sent privately to the action server and never
 * broadcast, so a passive observer structurally cannot see it (#393).
 *
 * Outcome is MoveIt's own space, not Mission's nav2 three-way split: SUCCESS maps to succeeded,
 * PREEMPTED to cancelled (the closest thing MoveIt has), and every other (necessarily negative)
 * `MoveItErrorCodes` to failed.
 */
class Manipulation : public dc_measurements::Measurement
{
public:
  Manipulation();
  ~Manipulation() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  using GetResult = moveit_msgs::action::MoveGroup::Impl::GetResultService;

  enum class GoalState : uint8_t
  {
    kOpen,          // Seen accepted/executing; a manipulation_start Record was emitted.
    kResultPending  // Seen terminal; get_result was requested but hasn't answered yet.
  };

  struct GoalTracking
  {
    rclcpp::Time accepted_stamp;
    GoalState state;
  };

  void statusCb(const action_msgs::msg::GoalStatusArray& msg);
  void onResult(const std::string& goal_key, const rclcpp::Time& accepted_stamp, const rclcpp::Time& terminal_stamp,
                rclcpp::Client<GetResult>::SharedFuture future);
  void enqueueEnd(const std::string& goal_key, const std::string& outcome, int32_t error_code, double planning_time,
                  double duration_sec, const rclcpp::Time& stamp);

  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Client<GetResult>::SharedPtr get_result_client_;
  std::string action_name_;
  std::string group_name_;

  // The status callback and the get_result response callback can land on different executor
  // threads under MeasurementServer's multi-threaded executor, and both touch what collect()
  // reads, so everything they share is guarded.
  mutable std::mutex mutex_;
  // Goals currently tracked, keyed by their UUID as a canonical hex string. A goal already
  // resolved (its manipulation_end Record emitted) is erased rather than kept as "done": the
  // status topic re-publishes its full known-goal list on every change, including goals that
  // finished long ago, and an unknown key seen in a terminal state is exactly how a stale
  // republish of an already-handled goal is told apart from one this instance never saw start --
  // both are simply skipped, which also means a goal whose accept this instance missed (started
  // watching mid-goal) never gets a synthesized start it can't back up.
  std::map<std::string, GoalTracking> goals_;
  std::deque<std::pair<json, rclcpp::Time>> pending_events_;
  // Per-Record, not per-goal: a global counter across every goal, so a consumer can tell a
  // dropped Record apart from one that was never produced.
  std::uint64_t record_seq_{ 0 };

protected:
  void onConfigure() override;
  void onCleanup() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MANIPULATION_HPP_
