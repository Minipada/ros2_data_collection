// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_FOLLOW_WAYPOINTS_TRACKER_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_FOLLOW_WAYPOINTS_TRACKER_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "dc_common/state_transition_detector.hpp"

namespace dc_measurements
{

/// Terminal nav2 GoalStatus values relevant to a mission, expressed with no ROS dependency --
/// ACCEPTED/EXECUTING/CANCELING never reach the tracker, only what a goal can end on.
enum class MissionTerminalStatus
{
  Succeeded,
  Canceled,
  Aborted,
};

/// The outcome a Record actually reports, per #387's schema. Distinct from MissionTerminalStatus:
/// a goal that reaches Succeeded with a non-zero error_code is reported as Failed rather than
/// Succeeded (nav2's WaypointFollower can finish its action goal successfully while still having
/// missed one or more waypoints).
enum class MissionOutcome
{
  Succeeded,
  Failed,
  Cancelled,
  Aborted,
};

/// One entry of FollowWaypoints::Result.missed_waypoints (nav2_msgs/msg/WaypointStatus), stripped
/// of the pose -- not part of #389's schema.
struct WaypointOutcome
{
  std::uint32_t index{ 0 };
  std::string status;  // "pending" | "completed" | "skipped" | "failed"
  std::uint16_t error_code{ 0 };
  std::string error_msg;
};

struct MissionStartFact
{
  std::string mission_id;
  std::uint64_t sequence{ 0 };
};

struct MissionEndFact
{
  std::string mission_id;
  std::uint64_t sequence{ 0 };
  MissionOutcome outcome{ MissionOutcome::Succeeded };
  double duration_sec{ 0.0 };
  /// Set exactly when outcome is Failed or Aborted, per #387's schema.
  std::optional<std::string> reason;
  std::optional<std::uint16_t> error_code;
  std::vector<WaypointOutcome> missed_waypoints;
};

/**
 * @class dc_measurements::MissionFollowWaypointsTracker
 * @brief Turns a passively observed FollowWaypoints goal lifecycle (accepted -> terminal) into
 * mission_start/mission_end facts, with no ROS dependency. Built on
 * dc_common::StateTransitionDetector<std::string> (#360), the same block `intervention`/`fault`
 * use, with the tracked "state" being the active goal_id (empty = idle).
 *
 * A nav2 WaypointFollower action server processes one goal at a time, so this tracker only ever
 * follows one mission at once; a second goal accepted before the first's result arrives is
 * reported by startMission() returning nullopt, for the caller to log and ignore.
 *
 * The terminal Result (error_code, error_msg, missed_waypoints) is not available at the moment a
 * goal reaches a terminal GoalStatus -- a passive watcher (mission_nav2_follow_waypoints.cpp) has
 * to fetch it with a separate, asynchronous get_result call. endMission() is therefore called once
 * that response has arrived, not from the status callback directly.
 */
class MissionFollowWaypointsTracker
{
public:
  using TimePoint = std::chrono::system_clock::time_point;

  /// `at` seeds the tracker as idle since that moment -- without it, the very first mission this
  /// instance ever observes would be silently absorbed as StateTransitionDetector's baseline
  /// sample rather than reported as a transition.
  explicit MissionFollowWaypointsTracker(TimePoint at) : detector_(std::string(), at)
  {
  }

  /// A new goal_id was accepted. nullopt when a mission is already tracked, or `goal_id` is
  /// already the one tracked (a duplicate status-array sighting).
  std::optional<MissionStartFact> startMission(const std::string& goal_id, TimePoint at)
  {
    if (detector_.state().value_or(std::string()) != std::string())
    {
      return std::nullopt;
    }
    const auto transition = detector_.update(goal_id, at);
    if (!transition.has_value())
    {
      return std::nullopt;
    }
    return MissionStartFact{ goal_id, transition->sequence };
  }

  /// The tracked goal's Result has arrived. nullopt when `goal_id` is not the one currently
  /// tracked (a stale or foreign result).
  std::optional<MissionEndFact> endMission(const std::string& goal_id, MissionTerminalStatus status,
                                           std::uint16_t error_code, std::string error_msg,
                                           std::vector<WaypointOutcome> missed_waypoints, TimePoint at)
  {
    if (detector_.state().value_or(std::string()) != goal_id || goal_id.empty())
    {
      return std::nullopt;
    }
    const auto transition = detector_.update(std::string(), at);
    if (!transition.has_value())
    {
      return std::nullopt;
    }

    MissionEndFact fact;
    fact.mission_id = goal_id;
    fact.sequence = transition->sequence;
    fact.duration_sec = std::chrono::duration<double>(transition->dwell).count();
    fact.missed_waypoints = std::move(missed_waypoints);

    if (status == MissionTerminalStatus::Aborted)
    {
      fact.outcome = MissionOutcome::Aborted;
    }
    else if (status == MissionTerminalStatus::Canceled)
    {
      fact.outcome = MissionOutcome::Cancelled;
    }
    else
    {
      // Succeeded at the action level, but nav2 still reports a non-zero error_code when it
      // stopped short of every waypoint -- an application-level failure, not collapsed into
      // Aborted (mirrors #387's NavigateToPose rule for the same situation).
      fact.outcome = (error_code != 0) ? MissionOutcome::Failed : MissionOutcome::Succeeded;
    }

    if (fact.outcome == MissionOutcome::Failed || fact.outcome == MissionOutcome::Aborted)
    {
      fact.reason = std::move(error_msg);
      fact.error_code = error_code;
    }
    return fact;
  }

  /// The goal_id currently tracked, or nullopt while idle -- what onCleanup() checks for a mission
  /// still open at shutdown.
  std::optional<std::string> activeMissionId() const
  {
    const auto& state = detector_.state();
    if (!state.has_value() || state->empty())
    {
      return std::nullopt;
    }
    return state;
  }

private:
  dc_common::StateTransitionDetector<std::string> detector_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_FOLLOW_WAYPOINTS_TRACKER_HPP_
