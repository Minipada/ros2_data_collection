// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_THROUGH_POSES_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_THROUGH_POSES_HPP_

#include <cstdint>
#include <map>
#include <string>

#include "dc_measurements/mission_action_watcher.hpp"
#include "dc_measurements/mission_uuid.hpp"
#include "dc_measurements/plugins/measurements/mission_nav2_through_poses_core.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

namespace dc_measurements
{

/**
 * @brief What NavigateThroughPoses (#388) contributes to the shared action-lifecycle adapter: the
 * MissionNav2ThroughPosesCore outcome core -- unlike #387/#389's single-goal trackers, it can
 * track several goals at once -- plus the per-goal `number_of_recoveries` capture (there is no
 * recovery count on the terminal Result, only on Feedback, so it is captured live and carried
 * forward to whichever mission_end Record eventually asks for it) and the plain-hex goal-id
 * mission_id format this variant already shipped with.
 */
struct MissionNav2ThroughPosesPolicy
{
  using TimePoint = std::chrono::system_clock::time_point;

  static constexpr const char* kMissionType = "navigate_through_poses";
  static constexpr const char* kDefaultActionName = "navigate_through_poses";
  static constexpr bool kWatchFeedback = true;
  static constexpr bool kRetryResultFetch = false;
  static constexpr const char* kCleanupStill = "running";
  static constexpr const char* kCleanupConsequence = "no mission_end Record will be published";

  void seed(TimePoint)
  {
  }

  /// A goal's very first sighting starts it, whatever the status -- unlike the single-goal
  /// trackers, MissionNav2ThroughPosesCore knows no idle baseline to differ against.
  std::optional<MissionStartFact> observe(const std::string& goal_id, std::int8_t, TimePoint at)
  {
    return core_.observe(goal_id, at);
  }

  /// Several goals can be tracked at once, so a refused start is never worth a warning.
  static bool busyWithAnotherMission(const std::string&)
  {
    return false;
  }

  bool shouldRequestResult(const std::string& goal_id)
  {
    return core_.shouldRequestResult(goal_id);
  }

  static void noteResultUnavailable(const std::string&)
  {
  }

  void noteFeedback(const std::string& goal_id, int number_of_recoveries)
  {
    last_recoveries_[goal_id] = number_of_recoveries;
  }

  static void noteResult(const nav2_msgs::action::NavigateThroughPoses::Impl::GetResultService::Response&)
  {
  }

  std::optional<MissionEndFact> end(const std::string& goal_id, std::int8_t status, std::uint16_t error_code,
                                    std::string error_msg, TimePoint at)
  {
    std::optional<int> recoveries;
    const auto recoveries_it = last_recoveries_.find(goal_id);
    if (recoveries_it != last_recoveries_.end())
    {
      recoveries = recoveries_it->second;
      last_recoveries_.erase(recoveries_it);
    }
    return core_.end(goal_id, static_cast<GoalPhase>(status), error_code, std::move(error_msg), recoveries, at);
  }

  std::vector<std::string> openMissionIds() const
  {
    return core_.openMissionIds();
  }

  static std::string goalKeyOf(const unique_identifier_msgs::msg::UUID& uuid)
  {
    return missionGoalIdHex(uuid.uuid);
  }

  static void enrichRecord(json& data, const MissionEndFact& fact)
  {
    if (fact.recoveries.has_value())
    {
      data["recoveries"] = *fact.recoveries;
    }
  }

  MissionNav2ThroughPosesCore core_;
  std::map<std::string, int> last_recoveries_;
};

/**
 * @class dc_measurements::MissionNav2ThroughPoses
 * @brief The nav2 adapter of the Mission Measurement (#396) for `NavigateThroughPoses`: the action
 * a deployment issues when a mission is a single job through several hard-constraint poses in one
 * call, rather than a chain of separate `NavigateToPose` goals (#387). Emits `mission_start` when a
 * goal is first observed and `mission_end` once it reaches a terminal state, in the same Record
 * schema #387 defines with `mission_type: "navigate_through_poses"`. The action-lifecycle wiring
 * is shared with the other nav2 Mission variants through MissionActionWatcher (#503).
 */
class MissionNav2ThroughPoses
  : public MissionActionWatcher<nav2_msgs::action::NavigateThroughPoses, MissionNav2ThroughPosesPolicy>
{
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_THROUGH_POSES_HPP_
