// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__MISSION_REGISTRY_HPP_
#define DC_MEASUREMENTS__MISSION_REGISTRY_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace dc_measurements
{

/// Bookkeeping shared by every Mission Measurement core that can track more than one mission at
/// once (MissionNav2ThroughPosesCore, MissionOpenRmfCore): a bounded id -> ActivePayload map,
/// oldest-finished-first eviction once `max_tracked` ids have ever been seen, and the monotonic
/// sequence counter every mission_start/mission_end fact carries. `ActivePayload` is whatever
/// per-mission state one core needs beyond "when did it start and is it finished" -- it must
/// expose a `bool finished` member; PrunedMissionMap never interprets any other field.
///
/// Owns no clock and no ROS/json type, matching every other tracker/core in this codebase: a
/// unit test drives it in microseconds with no node or action server involved.
template <typename ActivePayload>
class PrunedMissionMap
{
public:
  explicit PrunedMissionMap(std::size_t max_tracked = 256) : max_tracked_(max_tracked)
  {
  }

  ActivePayload* find(const std::string& id)
  {
    auto it = missions_.find(id);
    return it == missions_.end() ? nullptr : &it->second;
  }

  /// Starts tracking `id`. Caller must already have checked find(id) == nullptr -- mirrors every
  /// existing core's own observe()/end() call pattern, so this never silently overwrites an
  /// in-flight mission's state.
  ActivePayload& insert(const std::string& id, ActivePayload payload)
  {
    auto [it, inserted] = missions_.emplace(id, std::move(payload));
    order_.push_back(id);
    prune();
    return it->second;
  }

  std::uint64_t nextSequence()
  {
    return ++sequence_;
  }

  /// ids whose payload is not yet finished -- for onCleanup()'s "still running at shutdown"
  /// warning, the same shape every Mission core already reports it in.
  std::vector<std::string> openIds() const
  {
    std::vector<std::string> open;
    for (const auto& [id, payload] : missions_)
    {
      if (!payload.finished)
      {
        open.push_back(id);
      }
    }
    return open;
  }

private:
  // ids never repeat in practice (UUIDs / Open-RMF booking ids), so a long-running instance's map
  // would otherwise grow without bound; only ever prunes finished missions, oldest first.
  void prune()
  {
    while (order_.size() > max_tracked_)
    {
      const auto& oldest = order_.front();
      auto it = missions_.find(oldest);
      if (it != missions_.end() && it->second.finished)
      {
        missions_.erase(it);
        order_.pop_front();
      }
      else
      {
        // The oldest tracked id is still open (a very long-running mission) -- leave it rather
        // than dropping an active mission's tracking state.
        break;
      }
    }
  }

  std::size_t max_tracked_;
  std::map<std::string, ActivePayload> missions_;
  std::deque<std::string> order_;
  std::uint64_t sequence_{ 0 };
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__MISSION_REGISTRY_HPP_
