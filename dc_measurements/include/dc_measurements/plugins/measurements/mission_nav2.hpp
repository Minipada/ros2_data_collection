// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_HPP_

#include <optional>
#include <string>

#include "dc_measurements/mission_action_watcher.hpp"
#include "dc_measurements/mission_uuid.hpp"
#include "dc_measurements/plugins/measurements/mission_nav2_tracker.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace dc_measurements
{

/// What NavigateToPose (#387) contributes to the shared action-lifecycle adapter: the
/// MissionNav2Tracker outcome core, `recoveries` carried from Feedback onto the mission_end
/// Record, and the dashed goal-id mission_id format the contract was agreed against (#305).
struct MissionNav2Policy
  : SingleGoalMissionPolicy<nav2_msgs::action::NavigateToPose, MissionNav2Tracker, std::optional<int>>
{
  static constexpr const char* kMissionType = "navigate_to_pose";
  static constexpr const char* kDefaultActionName = "navigate_to_pose";
  static constexpr bool kWatchFeedback = true;
  static constexpr bool kRetryResultFetch = true;
  static constexpr const char* kCleanupStill = "active";
  static constexpr const char* kCleanupConsequence = "no closing Record will be published";

  static std::string goalKeyOf(const unique_identifier_msgs::msg::UUID& uuid)
  {
    return missionGoalIdDashed(uuid);
  }

  static void enrichRecord(json& data, const MissionEndFact& fact)
  {
    if (fact.recoveries.has_value())
    {
      data["recoveries"] = *fact.recoveries;
    }
  }
};

/**
 * @class dc_measurements::MissionNav2
 * @brief The nav2 adapter of the Mission Measurement (#396) for `NavigateToPose`: the base case
 * the mission lifecycle contract (#305, recorded in ADR-0010) was agreed against, with
 * `NavigateThroughPoses`/`FollowWaypoints` as its siblings (#388/#389). The action-lifecycle
 * wiring is shared with those siblings through MissionActionWatcher (#503).
 */
class MissionNav2 : public MissionActionWatcher<nav2_msgs::action::NavigateToPose, MissionNav2Policy>
{
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_HPP_
