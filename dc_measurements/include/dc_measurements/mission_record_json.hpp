// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__MISSION_RECORD_JSON_HPP_
#define DC_MEASUREMENTS__MISSION_RECORD_JSON_HPP_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "dc_measurements/mission_outcome.hpp"

namespace dc_measurements
{

/// The mission_start Record every Mission Measurement adapter emits, per #305/ADR-0010's shared
/// Record contract.
inline nlohmann::json missionStartJson(const std::string& mission_id, const std::string& mission_type,
                                       std::uint64_t sequence)
{
  nlohmann::json data;
  data["event"] = "mission_start";
  data["mission_id"] = mission_id;
  data["mission_type"] = mission_type;
  data["sequence"] = sequence;
  return data;
}

/// The mission_end Record fields common to every adapter (#305/ADR-0010). A caller with its own
/// extra fields (nav2's `recoveries`, FollowWaypoints' `missed_waypoints`) adds them to the
/// returned object before enqueuing it.
inline nlohmann::json missionEndJsonBase(const std::string& mission_id, const std::string& mission_type,
                                         std::uint64_t sequence, MissionOutcome outcome, double duration_sec,
                                         const std::optional<std::string>& reason,
                                         const std::optional<std::uint16_t>& error_code)
{
  nlohmann::json data;
  data["event"] = "mission_end";
  data["mission_id"] = mission_id;
  data["mission_type"] = mission_type;
  data["sequence"] = sequence;
  data["outcome"] = missionOutcomeName(outcome);
  data["duration_sec"] = duration_sec;
  if (reason.has_value())
  {
    data["reason"] = *reason;
  }
  if (error_code.has_value())
  {
    data["error_code"] = *error_code;
  }
  return data;
}

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__MISSION_RECORD_JSON_HPP_
