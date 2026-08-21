// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_TRACKER_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_TRACKER_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "dc_common/state_transition_detector.hpp"
#include "dc_measurements/mission_outcome.hpp"

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

struct MissionEndFact
{
  std::string mission_id;
  std::uint64_t sequence{ 0 };
  MissionOutcome outcome{ MissionOutcome::Succeeded };
  double duration_sec{ 0.0 };
  /// Set exactly when outcome is Failed or Aborted.
  std::optional<std::string> reason;
  std::optional<std::uint16_t> error_code;
  /// NavigateToPose::Feedback.number_of_recoveries at completion, when feedback was seen.
  std::optional<int> recoveries;
};

/**
 * @class dc_measurements::MissionNav2Tracker
 * @brief Turns a passively observed NavigateToPose goal lifecycle (accepted -> terminal) into
 * mission_start/mission_end facts, with no ROS dependency. Built on
 * dc_common::StateTransitionDetector<std::string> (#360), the same block `intervention`/`fault`
 * use, with the tracked "state" being the active goal_id (empty = idle) -- the pattern #389's
 * MissionFollowWaypointsTracker converged on, since a single-goal-at-a-time action server (nav2's
 * bt_navigator included) never needs more than one idle<->active transition pair in flight.
 *
 * nav2's NavigateToPose action server processes one goal at a time, so this tracker only ever
 * follows one mission at once; a second goal accepted before the first's result arrives is
 * reported by startMission() returning nullopt, for the caller to log and ignore.
 */
class MissionNav2Tracker
{
public:
  using TimePoint = std::chrono::system_clock::time_point;

  /// `at` seeds the tracker as idle since that moment -- without it, the very first mission this
  /// instance ever observes would be silently absorbed as StateTransitionDetector's baseline
  /// sample rather than reported as a transition.
  explicit MissionNav2Tracker(TimePoint at) : detector_(std::string(), at)
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
                                           std::optional<int> recoveries, TimePoint at)
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
    fact.recoveries = recoveries;

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
      // Succeeded at the action level, but nav2 still reports a non-zero error_code for an
      // application-level failure that did not abort the goal status itself.
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

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_TRACKER_HPP_
