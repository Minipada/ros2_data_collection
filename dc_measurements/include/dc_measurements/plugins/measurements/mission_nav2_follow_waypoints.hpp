// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_FOLLOW_WAYPOINTS_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_FOLLOW_WAYPOINTS_HPP_

#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/mission_record_json.hpp"
#include "dc_measurements/pending_record_queue.hpp"
#include "dc_measurements/plugins/measurements/mission_follow_waypoints_tracker.hpp"
#include "dc_util/node_utils.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace dc_measurements
{

/**
 * @class dc_measurements::MissionNav2FollowWaypoints
 * @brief The FollowWaypoints sibling of #387's nav2 Mission Measurement, reusing its
 * mission_start/mission_end Record schema and mission_id/sequence conventions.
 *
 * A passive watcher, not a commander: nav2's WaypointFollower action is driven by whatever already
 * dispatches missions on the robot (nav2_simple_commander, a WMS integration, ...) -- DC never
 * sends a FollowWaypoints goal of its own, matching every other Measurement's read-only relationship
 * to the systems it observes. That rules out `rclcpp_action::Client`'s typed goal-tracking API,
 * which only reports on goals the client itself sent: this subscribes directly to the action's
 * `_action/status` topic (`action_msgs/msg/GoalStatusArray`, the same type for every action) for
 * accept/terminal-state boundaries, and calls its `_action/get_result` service directly for the
 * terminal Result once a goal it is following reaches one -- both public, standard parts of the
 * ROS 2 action wire protocol that any client may use, sender or not.
 *
 * One consequence: `number_of_loops` (#389's acceptance criteria) is a Goal-only field. ROS 2's
 * action protocol never re-publishes the Goal anywhere a third party can observe it -- only the
 * sender and the server ever see it, and rclcpp_action::Client has no supported way to attach to a
 * goal it did not send in order to read it back either. It is therefore not present on the Records
 * this plugin emits; see doc/src/dc/measurements/mission_nav2_follow_waypoints.md.
 */
class MissionNav2FollowWaypoints : public dc_measurements::Measurement
{
public:
  using GetResultService = nav2_msgs::action::FollowWaypoints::Impl::GetResultService;

  MissionNav2FollowWaypoints();
  ~MissionNav2FollowWaypoints() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  void statusCb(const action_msgs::msg::GoalStatusArray& msg);
  void requestResult(const unique_identifier_msgs::msg::UUID& uuid, const std::string& goal_id,
                     const rclcpp::Time& terminal_at);
  void handleResultResponse(const std::string& goal_id, const rclcpp::Time& terminal_at,
                            const GetResultService::Response::SharedPtr& response);
  // Caller holds mutex_.
  void enqueue(json data, const rclcpp::Time& stamp);

  std::string action_name_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Client<GetResultService>::SharedPtr result_client_;

  // The status callback and the get_result response callback run on different callback groups
  // than the polling timer under a multi-threaded executor, so everything they share is guarded.
  mutable std::mutex mutex_;
  std::optional<MissionFollowWaypointsTracker> tracker_;
  // Which goal_ids a get_result request is already in flight for, so a terminal status repeated
  // across several status-array publishes doesn't fire the service call more than once.
  std::set<std::string> result_requested_;
  // Records wait here for a poll to carry them out, one per poll, so they travel the same publish
  // path (Conditions, buffering, Group) as every other Record.
  PendingRecordQueue pending_records_;

protected:
  void onConfigure() override;
  void onCleanup() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_FOLLOW_WAYPOINTS_HPP_
