// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_OPEN_RMF_CORE_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_OPEN_RMF_CORE_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "dc_measurements/mission_outcome.hpp"
#include "dc_measurements/mission_registry.hpp"

namespace dc_measurements
{

/// Mirrors `rmf_api_msgs`' `task_state.json` `status` enum verbatim (12 values, in schema order)
/// -- this header stays free of any rmf/ROS include, so the plugin is the only place a raw status
/// string is ever parsed. `UnknownStatus` is not one of Open-RMF's own values; it is what an
/// unrecognized string parses to, so a status this Measurement doesn't understand yet is inert
/// rather than misclassified as pre-start, active, or terminal.
enum class RmfTaskStatus : std::uint8_t
{
  Uninitialized,
  Blocked,
  Error,
  Failed,
  Queued,
  Standby,
  Underway,
  Delayed,
  Skipped,
  Canceled,
  Killed,
  Completed,
  UnknownStatus,
};

/// Parses one of `task_state.json`'s `status` enum strings. Any other string (including one this
/// version of the schema doesn't define yet) maps to `UnknownStatus`.
inline RmfTaskStatus parseRmfTaskStatus(const std::string& status)
{
  static const std::map<std::string, RmfTaskStatus> kByName = {
    { "uninitialized", RmfTaskStatus::Uninitialized },
    { "blocked", RmfTaskStatus::Blocked },
    { "error", RmfTaskStatus::Error },
    { "failed", RmfTaskStatus::Failed },
    { "queued", RmfTaskStatus::Queued },
    { "standby", RmfTaskStatus::Standby },
    { "underway", RmfTaskStatus::Underway },
    { "delayed", RmfTaskStatus::Delayed },
    { "skipped", RmfTaskStatus::Skipped },
    { "canceled", RmfTaskStatus::Canceled },
    { "killed", RmfTaskStatus::Killed },
    { "completed", RmfTaskStatus::Completed },
  };
  const auto it = kByName.find(status);
  return it == kByName.end() ? RmfTaskStatus::UnknownStatus : it->second;
}

struct MissionEndFact
{
  std::string mission_id;
  std::uint64_t sequence;
  MissionOutcome outcome;
  double duration_sec;
  std::optional<std::string> reason;
  std::optional<std::uint16_t> error_code;
};

/// One observed `TaskState` sample, already reduced to what MissionOpenRmfCore needs -- the
/// plugin does all the `nlohmann::json` field digging (booking.id, category, cancellation/killed
/// labels, dispatch.errors, ...) and hands this ROS/json-free struct across, same split as
/// dc_common::BatteryCycleAccumulator/StateTransitionDetector and #388's
/// MissionNav2ThroughPosesCore.
struct TaskStateSample
{
  std::string mission_id;
  std::string mission_type;
  RmfTaskStatus status{ RmfTaskStatus::UnknownStatus };
  /// Open-RMF's own `unix_millis_start_time`/`unix_millis_finish_time`, when the source provided
  /// both -- preferred over locally observed timestamps for `duration_sec` when available (#391's
  /// "carry the source's own values" approach, extended from #387's message/error_code precedent
  /// to timestamps).
  std::optional<std::int64_t> unix_millis_start_time;
  std::optional<std::int64_t> unix_millis_finish_time;
  /// Best-effort human-readable detail already extracted from whichever field Open-RMF actually
  /// populated for this status (`cancellation.labels`, `killed.labels`, `dispatch.errors`, or the
  /// top-level `detail`) -- not guaranteed present for every terminal status, unlike nav2's
  /// always-populated `error_msg`.
  std::optional<std::string> reason;
  std::optional<std::uint16_t> error_code;
};

/**
 * @class dc_measurements::MissionOpenRmfCore
 * @brief ROS-free interpretation of one Open-RMF task's lifecycle (#391), following #388's split
 * precedent: which `mission_id` (Open-RMF's `booking.id`) is newly active (a mission_start fact)
 * and, once its status is terminal, what its mission_end fact looks like (outcome, duration,
 * reason). Owns no clock and no `nlohmann::json`, so a unit test drives a whole task's lifecycle
 * in microseconds with no node, no websocket, and no JSON parsing involved.
 *
 * Two judgment calls this class encodes, per #391's acceptance criteria asking for them to be
 * resolved explicitly rather than silently defaulted:
 *
 * - **`skipped` is treated as task-terminal**, not merely a transient phase-level status: it is
 *   defined in the exact same `status` enum `rmf_api_msgs`' `task_state.json` schema uses for the
 *   task's own top-level `status` field (not a separate phase-only enum), so a task can
 *   legitimately end its life with `status: "skipped"`. It maps to outcome `cancelled` --
 *   Open-RMF's `skipped` means the task's work was bypassed rather than performed, which reads
 *   closer to DC's `cancelled` (closed without completing the work, not an error) than to
 *   `succeeded` (implies the work was done) or a failure outcome. DC's outcome contract (#305/
 *   #387) has four values, not five; this Measurement does not add a `skipped` outcome to it.
 * - **`blocked` and `error` are treated as transient, not terminal.** Both describe a task that is
 *   currently unable to make progress while Open-RMF keeps trying to resolve or recover it (a
 *   blocked path, a recoverable fault) rather than a status Open-RMF's own task manager ever
 *   settles on as the end of the task's life; they are absent from `task_state.json`'s modeled
 *   terminal set (`failed`/`canceled`/`killed`/`completed`/`skipped`). A task observed as
 *   `blocked`/`error` stays open here; only a later terminal status closes it.
 *
 * The bounded id -> state bookkeeping (oldest-finished-first eviction, the sequence counter) is
 * PrunedMissionMap, shared with MissionNav2ThroughPosesCore -- the two cores differ only in what a
 * tracked mission's Active payload holds and how a terminal outcome is derived, which stays here.
 */
class MissionOpenRmfCore
{
public:
  using TimePoint = std::chrono::system_clock::time_point;

  struct ObserveResult
  {
    std::optional<MissionStartFact> start;
    std::optional<MissionEndFact> end;
  };

  /**
   * @brief Feed one observed `TaskState` sample. Returns a MissionStartFact the first time
   * `sample.mission_id` is seen in an active status (`underway`/`delayed`/`blocked`/`error` --
   * i.e. having left `queued`/`standby` behind, per #391's acceptance criteria), and a
   * MissionEndFact once a mission_id already tracked as started reaches a terminal status.
   *
   * A `mission_id` whose first-ever observed sample is already pre-start (`uninitialized`/
   * `queued`/`standby`) or already terminal is not tracked at all: no mission_start Record (the
   * active-transition boundary this Measurement watches for was never observed) and consequently
   * no mission_end Record either, symmetric with the open-interval case below and with #387's
   * "nothing downstream can average an interval that was never fully observed" reasoning.
   */
  ObserveResult observe(const TaskStateSample& sample, TimePoint at)
  {
    ObserveResult result;
    auto* active = registry_.find(sample.mission_id);

    if (active == nullptr)
    {
      if (!isActive(sample.status))
      {
        return result;
      }
      active = &registry_.insert(sample.mission_id, Active{ at, false });
      result.start = MissionStartFact{ sample.mission_id, registry_.nextSequence() };
    }

    if (active->finished)
    {
      return result;
    }

    if (isTerminal(sample.status))
    {
      MissionEndFact fact;
      fact.mission_id = sample.mission_id;
      fact.sequence = registry_.nextSequence();
      fact.outcome = outcomeFor(sample.status);
      fact.reason = sample.reason;
      fact.error_code = sample.error_code;

      if (sample.unix_millis_start_time.has_value() && sample.unix_millis_finish_time.has_value())
      {
        fact.duration_sec =
            static_cast<double>(*sample.unix_millis_finish_time - *sample.unix_millis_start_time) / 1000.0;
      }
      else
      {
        const TimePoint started_at = active->started_at;
        fact.duration_sec =
            std::chrono::duration<double>(at > started_at ? at - started_at : TimePoint::duration::zero()).count();
      }

      active->finished = true;
      result.end = fact;
    }

    return result;
  }

  /// mission_ids still open (mission_start emitted, no mission_end yet) -- for onCleanup()'s
  /// shutdown warning, mirroring MissionNav2ThroughPosesCore::openMissionIds().
  std::vector<std::string> openMissionIds() const
  {
    return registry_.openIds();
  }

private:
  struct Active
  {
    TimePoint started_at;
    bool finished{ false };
  };

  static bool isActive(RmfTaskStatus status)
  {
    switch (status)
    {
      case RmfTaskStatus::Underway:
      case RmfTaskStatus::Delayed:
      case RmfTaskStatus::Blocked:
      case RmfTaskStatus::Error:
        return true;
      default:
        return false;
    }
  }

  static bool isTerminal(RmfTaskStatus status)
  {
    switch (status)
    {
      case RmfTaskStatus::Completed:
      case RmfTaskStatus::Failed:
      case RmfTaskStatus::Canceled:
      case RmfTaskStatus::Killed:
      case RmfTaskStatus::Skipped:
        return true;
      default:
        return false;
    }
  }

  static MissionOutcome outcomeFor(RmfTaskStatus status)
  {
    switch (status)
    {
      case RmfTaskStatus::Completed:
        return MissionOutcome::Succeeded;
      case RmfTaskStatus::Failed:
        return MissionOutcome::Failed;
      case RmfTaskStatus::Killed:
        return MissionOutcome::Aborted;
      case RmfTaskStatus::Canceled:
      case RmfTaskStatus::Skipped:
      default:
        return MissionOutcome::Cancelled;
    }
  }

  PrunedMissionMap<Active> registry_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__MISSION_OPEN_RMF_CORE_HPP_
