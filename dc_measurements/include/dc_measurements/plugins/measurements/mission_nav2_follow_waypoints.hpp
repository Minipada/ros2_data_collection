// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_FOLLOW_WAYPOINTS_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_FOLLOW_WAYPOINTS_HPP_

#include <string>
#include <vector>

#include "dc_measurements/mission_action_watcher.hpp"
#include "dc_measurements/mission_uuid.hpp"
#include "dc_measurements/plugins/measurements/mission_follow_waypoints_tracker.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/msg/missed_waypoint.hpp"

namespace dc_measurements
{

/// What FollowWaypoints (#389) contributes to the shared action-lifecycle adapter: the
/// MissionFollowWaypointsTracker outcome core, the missed-waypoint list carried from the terminal
/// Result onto the mission_end Record (its Feedback has no recovery count to watch), and the same
/// dashed goal-id mission_id format as #387.
struct MissionNav2FollowWaypointsPolicy
  : SingleGoalMissionPolicy<nav2_msgs::action::FollowWaypoints, MissionFollowWaypointsTracker,
                            std::vector<WaypointOutcome>>
{
  static constexpr const char* kMissionType = "follow_waypoints";
  static constexpr const char* kDefaultActionName = "follow_waypoints";
  static constexpr bool kWatchFeedback = false;
  static constexpr bool kRetryResultFetch = true;
  static constexpr const char* kCleanupStill = "active";
  static constexpr const char* kCleanupConsequence = "no closing Record will be published";

  static std::string goalKeyOf(const unique_identifier_msgs::msg::UUID& uuid)
  {
    return missionGoalIdDashed(uuid);
  }

  static void enrichRecord(json& data, const MissionEndFact& fact)
  {
    json missed_json = json::array();
    for (const auto& waypoint : fact.missed_waypoints)
    {
      missed_json.push_back({ { "index", waypoint.index }, { "error_code", waypoint.error_code } });
    }
    data["missed_waypoints"] = missed_json;
  }
};

/**
 * @class dc_measurements::MissionNav2FollowWaypoints
 * @brief The FollowWaypoints sibling of #387's nav2 Mission Measurement, reusing its
 * mission_start/mission_end Record schema and mission_id/sequence conventions. The action-lifecycle
 * wiring is shared with the other nav2 Mission variants through MissionActionWatcher (#503).
 *
 * A passive watcher, not a commander: nav2's WaypointFollower action is driven by whatever already
 * dispatches missions on the robot (nav2_simple_commander, a WMS integration, ...) -- DC never
 * sends a FollowWaypoints goal of its own, matching every other Measurement's read-only relationship
 * to the systems it observes.
 *
 * One consequence: `number_of_loops` (#389's acceptance criteria) is a Goal-only field. ROS 2's
 * action protocol never re-publishes the Goal anywhere a third party can observe it -- only the
 * sender and the server ever see it, and rclcpp_action::Client has no supported way to attach to a
 * goal it did not send in order to read it back either. It is therefore not present on the Records
 * this plugin emits; see doc/src/dc/measurements/mission_nav2_follow_waypoints.md.
 */
class MissionNav2FollowWaypoints
  : public MissionActionWatcher<nav2_msgs::action::FollowWaypoints, MissionNav2FollowWaypointsPolicy>
{
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_FOLLOW_WAYPOINTS_HPP_
