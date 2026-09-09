// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__SOURCE_ADAPTER_HPP_
#define DC_MEASUREMENTS__SOURCE_ADAPTER_HPP_

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace dc_measurements
{

/// The one handoff between a Measurement's sources and its polling timer: a subscription (or a
/// service-client response, a websocket read -- any thread) hands a decoded value over with
/// push(), the poll drains it. Both sides run concurrently -- the poll timer sits in a
/// Reentrant callback group (measurement.hpp) beside the subscriptions, and the action/websocket
/// adapters hear back on their own client/IO threads -- so the adapter owns the mutex every
/// plugin used to hand-roll around this (cmd_vel/speed had none: a live data race).
///
/// The backlog is bounded by `capacity`; past it push() drops the OLDEST entry -- the most
/// recent values are the ones still worth reporting -- and returns true so the caller logs its
/// own measurement-name-specific warning. Capacity 1 is the keep-only-the-latest shape: every
/// push displaces the previous value.
///
/// Two read shapes, by source kind:
/// - an event source (a fault raised, a mission ended) drains one value per poll with pop();
///   nothing is re-reported once taken.
/// - a sample source (battery state, camera frame) re-reads the newest value on every poll
///   with latest(), which copies without draining.
template <typename T>
class SourceAdapter
{
public:
  explicit SourceAdapter(std::size_t capacity = 1) : capacity_(capacity)
  {
  }

  /// Hands one decoded value over from a source thread. Returns true when this push dropped
  /// the oldest queued entry to stay within `capacity`, so the caller can log its own
  /// throttled warning.
  bool push(T value)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    values_.emplace_back(std::move(value));
    if (values_.size() > capacity_)
    {
      values_.pop_front();
      return true;
    }
    return false;
  }

  /// Drains the oldest pending value, one per poll. nullopt when nothing arrived since the
  /// last drain.
  std::optional<T> pop()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (values_.empty())
    {
      return std::nullopt;
    }
    T value = std::move(values_.front());
    values_.pop_front();
    return value;
  }

  /// The newest pending value, copied -- for a sample source that re-reports it on every poll
  /// until a newer one lands. Does not drain: pop() still hands it out afterwards.
  std::optional<T> latest() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (values_.empty())
    {
      return std::nullopt;
    }
    return values_.back();
  }

private:
  std::size_t capacity_;
  mutable std::mutex mutex_;
  std::deque<T> values_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__SOURCE_ADAPTER_HPP_
