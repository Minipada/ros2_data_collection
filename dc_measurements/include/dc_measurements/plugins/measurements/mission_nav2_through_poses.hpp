// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_THROUGH_POSES_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_THROUGH_POSES_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "action_msgs/msg/goal_status_array.hpp"
#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/plugins/measurements/mission_nav2_through_poses_core.hpp"
#include "dc_util/node_utils.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace dc_measurements
{

/**
 * @class dc_measurements::MissionNav2ThroughPoses
 * @brief The nav2 adapter of the Mission Measurement (#396) for `NavigateThroughPoses`: the action
 * a deployment issues when a mission is a single job through several hard-constraint poses in one
 * call, rather than a chain of separate `NavigateToPose` goals (#387). Emits `mission_start` when a
 * goal is first observed and `mission_end` once it reaches a terminal state, in the same Record
 * schema #387 defines with `mission_type: "navigate_through_poses"`.
 *
 * A passive observer, not a second client competing for the action server's single active goal:
 * it subscribes to the action's own status (`_action/status`) and feedback (`_action/feedback`)
 * topics and calls its `_action/get_result` service directly for whichever goal_id just reached a
 * terminal status -- the same standard per-action topics/services every `rclcpp_action::Server`
 * exposes, rather than `rclcpp_action::Client`'s higher-level API, which has no supported way to
 * attach to a goal this Measurement did not itself send.
 */
class MissionNav2ThroughPoses : public dc_measurements::Measurement
{
public:
  using ActionT = nav2_msgs::action::NavigateThroughPoses;
  using GetResultSrv = ActionT::Impl::GetResultService;
  using FeedbackMsg = ActionT::Impl::FeedbackMessage;

  MissionNav2ThroughPoses();
  ~MissionNav2ThroughPoses() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  void statusCb(const action_msgs::msg::GoalStatusArray& msg);
  void feedbackCb(const FeedbackMsg& msg);
  void requestResult(const std::string& goal_id_hex, const unique_identifier_msgs::msg::UUID& goal_id, GoalPhase phase,
                     const rclcpp::Time& detected_at);
  void handleResult(const std::string& goal_id_hex, GoalPhase phase, const rclcpp::Time& detected_at,
                    const GetResultSrv::Response::SharedPtr& response);
  void emit(json data, const rclcpp::Time& stamp);

  std::string action_name_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
  rclcpp::Client<GetResultSrv>::SharedPtr get_result_client_;

  // The status/feedback callbacks and the polling timer run in different callback groups under a
  // multi-threaded executor, so everything they share is guarded -- same reasoning as
  // Battery/Fault.
  mutable std::mutex mutex_;
  MissionNav2ThroughPosesCore core_;
  // Last-seen `number_of_recoveries` per goal_id, from the feedback topic -- there is no recovery
  // count on the terminal Result, only on Feedback, so it has to be captured live and carried
  // forward to whichever mission_end Record eventually asks for it.
  std::map<std::string, int> last_recoveries_;
  // Records wait here for a poll to carry them out, one per poll, so they travel the same publish
  // path (Conditions, buffering, Group) as every other Record.
  std::deque<std::pair<json, rclcpp::Time>> pending_records_;

protected:
  void onConfigure() override;
  void onCleanup() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_THROUGH_POSES_HPP_
