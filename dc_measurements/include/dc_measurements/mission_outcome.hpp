// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__MISSION_OUTCOME_HPP_
#define DC_MEASUREMENTS__MISSION_OUTCOME_HPP_

#include <cstdint>
#include <string>

namespace dc_measurements
{

/// The outcome a Mission Record reports on mission_end, per the mission lifecycle contract
/// (#305, recorded in ADR-0010). Shared by every Mission Measurement adapter (nav2's three
/// action variants, Open-RMF) so the vocabulary -- and what a caller can do with it -- stays one
/// type rather than four independently-declared look-alikes.
enum class MissionOutcome
{
  Succeeded,
  Failed,
  Cancelled,
  Aborted,
};

/// The JSON-Record spelling of a MissionOutcome, per #305/ADR-0010's Record shape.
inline std::string missionOutcomeName(MissionOutcome outcome)
{
  switch (outcome)
  {
    case MissionOutcome::Succeeded:
      return "succeeded";
    case MissionOutcome::Failed:
      return "failed";
    case MissionOutcome::Cancelled:
      return "cancelled";
    case MissionOutcome::Aborted:
      return "aborted";
  }
  return "unknown";
}

/// A mission_start fact: identical across every adapter (nav2's three action variants,
/// Open-RMF) -- only what varies per source (the terminal outcome, its reason, extra fields
/// like recoveries/missed_waypoints) lives in each adapter's own MissionEndFact.
struct MissionStartFact
{
  std::string mission_id;
  std::uint64_t sequence{ 0 };
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__MISSION_OUTCOME_HPP_
