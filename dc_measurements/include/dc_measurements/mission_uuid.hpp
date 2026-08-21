// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__MISSION_UUID_HPP_
#define DC_MEASUREMENTS__MISSION_UUID_HPP_

#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include "unique_identifier_msgs/msg/uuid.hpp"

namespace dc_measurements
{

/// A goal UUID as RFC-4122-style dashed lowercase hex (`xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx`).
/// Used as `mission_id` by the single-goal-at-a-time nav2 adapters (NavigateToPose,
/// FollowWaypoints), which format directly from the action's own UUID message.
inline std::string missionGoalIdDashed(const unique_identifier_msgs::msg::UUID& uuid)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < uuid.uuid.size(); ++i)
  {
    oss << std::setw(2) << static_cast<int>(uuid.uuid[i]);
    if (i == 3 || i == 5 || i == 7 || i == 9)
    {
      oss << '-';
    }
  }
  return oss.str();
}

/// A goal UUID as plain (non-dashed) lowercase hex. Used as `mission_id` by
/// MissionNav2ThroughPoses, which formats from `goal_info.goal_id.uuid` directly rather than the
/// full UUID message type. Kept distinct from missionGoalIdDashed() rather than unified: an
/// already-shipped adapter's mission_id format is a Record-shape detail downstream consumers may
/// already depend on, not an implementation detail free to change underneath them.
inline std::string missionGoalIdHex(const std::array<std::uint8_t, 16>& uuid)
{
  std::ostringstream oss;
  for (auto byte : uuid)
  {
    oss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(byte);
  }
  return oss.str();
}

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__MISSION_UUID_HPP_
