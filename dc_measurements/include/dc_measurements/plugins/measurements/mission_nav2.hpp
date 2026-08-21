// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_HPP_

#include <mutex>
#include <optional>
#include <set>
#include <string>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/mission_record_json.hpp"
#include "dc_measurements/pending_record_queue.hpp"
#include "dc_measurements/plugins/measurements/mission_nav2_tracker.hpp"
#include "dc_util/node_utils.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace dc_measurements
{

/**
 * @class dc_measurements::MissionNav2
 * @brief The nav2 adapter of the Mission Measurement (#396) for `NavigateToPose`: the base case
 * the mission lifecycle contract (#305, recorded in ADR-0010) was agreed against, with
 * `NavigateThroughPoses`/`FollowWaypoints` as its siblings (#388/#389).
 *
 * A passive watcher, not a commander, matching #388/#389's design: nav2's `NavigateToPose` action
 * is driven by whatever already dispatches missions on the robot (the BT navigator, a fleet
 * orchestrator, an operator command) -- DC never sends a goal of its own. This subscribes directly
 * to the action's `_action/status`/`_action/feedback` topics and calls its `_action/get_result`
 * service for the terminal Result once a goal it is following reaches a terminal state -- the same
 * standard, public per-action topics/services every `rclcpp_action::Server` exposes, rather than
 * `rclcpp_action::Client`'s typed API, which only reports on goals the client itself sent.
 */
class MissionNav2 : public dc_measurements::Measurement
{
public:
  using ActionT = nav2_msgs::action::NavigateToPose;
  using GetResultService = ActionT::Impl::GetResultService;
  using FeedbackMsg = ActionT::Impl::FeedbackMessage;

  MissionNav2();
  ~MissionNav2() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  void statusCb(const action_msgs::msg::GoalStatusArray& msg);
  void feedbackCb(const FeedbackMsg& msg);
  void requestResult(const unique_identifier_msgs::msg::UUID& uuid, const std::string& goal_id,
                     const rclcpp::Time& terminal_at);
  void handleResultResponse(const std::string& goal_id, const rclcpp::Time& terminal_at,
                            const GetResultService::Response::SharedPtr& response);
  // Caller holds mutex_.
  void enqueue(json data, const rclcpp::Time& stamp);

  std::string action_name_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
  rclcpp::Client<GetResultService>::SharedPtr result_client_;

  // The status/feedback callbacks and the get_result response callback run on different callback
  // groups than the polling timer under a multi-threaded executor, so everything they share is
  // guarded.
  mutable std::mutex mutex_;
  std::optional<MissionNav2Tracker> tracker_;
  // Which goal_ids a get_result request is already in flight for, so a terminal status repeated
  // across several status-array publishes doesn't fire the service call more than once.
  std::set<std::string> result_requested_;
  // Last-seen number_of_recoveries for the mission currently tracked, from the feedback topic --
  // there is no recovery count on the terminal Result, only on Feedback.
  std::optional<int> last_recoveries_;
  // Records wait here for a poll to carry them out, one per poll, so they travel the same publish
  // path (Conditions, buffering, Group) as every other Record.
  PendingRecordQueue pending_records_;

protected:
  void onConfigure() override;
  void onCleanup() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_HPP_
