// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_THROUGH_POSES_CORE_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_THROUGH_POSES_CORE_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "dc_measurements/mission_outcome.hpp"
#include "dc_measurements/mission_registry.hpp"

namespace dc_measurements
{

/// Mirrors action_msgs::msg::GoalStatus's STATUS_* constants numerically, so this header (and its
/// unit test) stay free of any action_msgs/ROS include -- the plugin static_casts the real message
/// field into this enum.
enum class GoalPhase : std::uint8_t
{
  Unknown = 0,
  Accepted = 1,
  Executing = 2,
  Canceling = 3,
  Succeeded = 4,
  Canceled = 5,
  Aborted = 6,
};

struct MissionEndFact
{
  std::string mission_id;
  std::uint64_t sequence;
  MissionOutcome outcome;
  double duration_sec;
  std::optional<std::string> reason;
  std::optional<std::uint16_t> error_code;
  std::optional<int> recoveries;
};

/**
 * @class dc_measurements::MissionNav2ThroughPosesCore
 * @brief ROS-free interpretation of a NavigateThroughPoses goal's lifecycle, following #360's
 * split precedent (dc_common::BatteryCycleAccumulator/StateTransitionDetector): which goal_id is
 * new (a mission_start fact) and, once its terminal status and result are known, what its
 * mission_end fact looks like (outcome, duration, reason). Owns no clock and no ROS types, so a
 * unit test drives a whole mission in microseconds with no node or action server involved.
 *
 * Unlike dc_common::StateTransitionDetector, a mission's "start" is the very first sample seen for
 * a goal_id, not a transition away from some prior baseline -- there is no natural idle state a
 * goal_id holds before it exists -- so this is a small bespoke tracker rather than a
 * StateTransitionDetector<GoalPhase> instantiation.
 *
 * The bounded id -> state bookkeeping (oldest-finished-first eviction, the sequence counter) is
 * PrunedMissionMap, shared with MissionOpenRmfCore -- the two cores differ only in what a tracked
 * mission's Active payload holds and how a terminal outcome is derived, which stays here.
 */
class MissionNav2ThroughPosesCore
{
public:
  using TimePoint = std::chrono::system_clock::time_point;

  /**
   * @brief Feed one observed status sample for `goal_id`. Returns a MissionStartFact the first
   * time this goal_id is seen and nullopt on every later sample for the same goal_id -- only the
   * boundary is reported, matching every other tracker in this codebase.
   */
  std::optional<MissionStartFact> observe(const std::string& goal_id, TimePoint at)
  {
    if (registry_.find(goal_id) != nullptr)
    {
      return std::nullopt;
    }
    registry_.insert(goal_id, Active{ at, false, false });
    return MissionStartFact{ goal_id, registry_.nextSequence() };
  }

  /**
   * @brief Whether the caller should fetch `goal_id`'s result now that it has reached a terminal
   * phase. True only the first time a terminal phase is observed for a goal_id, so a status topic
   * that keeps republishing a finished goal's last entry doesn't queue duplicate result fetches.
   */
  bool shouldRequestResult(const std::string& goal_id)
  {
    auto* active = registry_.find(goal_id);
    if (active == nullptr || active->finished || active->result_requested)
    {
      return false;
    }
    active->result_requested = true;
    return true;
  }

  /**
   * @brief `goal_id`'s mission_end fact, once its terminal phase and result are known. `outcome`
   * follows #387/#388's contract: CANCELED maps to cancelled, ABORTED to aborted (both carrying
   * `error_msg`/`error_code` as `reason`/`error_code`), and SUCCEEDED maps to succeeded unless
   * `error_code` is non-zero, in which case it is failed (also carrying reason/error_code) -- an
   * application-level failure nav2 reported without aborting the goal status itself.
   */
  MissionEndFact end(const std::string& goal_id, GoalPhase terminal_phase, std::uint16_t error_code,
                     const std::string& error_msg, std::optional<int> recoveries, TimePoint at)
  {
    MissionEndFact fact;
    fact.mission_id = goal_id;
    fact.sequence = registry_.nextSequence();
    fact.recoveries = recoveries;

    auto* active = registry_.find(goal_id);
    const TimePoint started_at = (active != nullptr) ? active->started_at : at;
    fact.duration_sec =
        std::chrono::duration<double>(at > started_at ? at - started_at : TimePoint::duration::zero()).count();

    switch (terminal_phase)
    {
      case GoalPhase::Canceled:
        fact.outcome = MissionOutcome::Cancelled;
        break;
      case GoalPhase::Aborted:
        fact.outcome = MissionOutcome::Aborted;
        fact.reason = error_msg;
        fact.error_code = error_code;
        break;
      case GoalPhase::Succeeded:
      default:
        if (error_code != 0)
        {
          fact.outcome = MissionOutcome::Failed;
          fact.reason = error_msg;
          fact.error_code = error_code;
        }
        else
        {
          fact.outcome = MissionOutcome::Succeeded;
        }
        break;
    }

    if (active != nullptr)
    {
      active->finished = true;
    }
    return fact;
  }

  /// goal_ids still open (mission_start emitted, no mission_end yet) -- for onCleanup()'s shutdown
  /// warning, mirroring Fault's fault_started_at_ map.
  std::vector<std::string> openMissionIds() const
  {
    return registry_.openIds();
  }

private:
  struct Active
  {
    TimePoint started_at;
    bool finished{ false };
    bool result_requested{ false };
  };

  PrunedMissionMap<Active> registry_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_NAV2_THROUGH_POSES_CORE_HPP_
