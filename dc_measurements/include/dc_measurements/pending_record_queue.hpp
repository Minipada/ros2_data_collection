// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PENDING_RECORD_QUEUE_HPP_
#define DC_MEASUREMENTS__PENDING_RECORD_QUEUE_HPP_

#include <cstddef>
#include <deque>
#include <nlohmann/json.hpp>
#include <utility>

#include "rclcpp/time.hpp"

namespace dc_measurements
{

/// A bounded FIFO of collected-but-not-yet-published Records, drained one per poll via pop() --
/// the shape every Measurement that can emit more than one Record between polls (Fault,
/// Ros2ControlStatus, the Mission adapters) already needed, previously reimplemented per plugin.
/// One Record leaves per poll, so a source producing Records far faster than the polling interval
/// would otherwise queue without bound: past `max_size`, push() drops the oldest entry -- the
/// most recent boundaries are the ones still worth reporting -- and reports that back to the
/// caller, which owns the actual logger/measurement-name-specific warning text.
class PendingRecordQueue
{
public:
  explicit PendingRecordQueue(std::size_t max_size = 64) : max_size_(max_size)
  {
  }

  /// Queues `data` for emission at `stamp`. Returns true when this push dropped the oldest queued
  /// entry to stay within `max_size`, so the caller can log its own throttled warning.
  bool push(nlohmann::json data, const rclcpp::Time& stamp)
  {
    records_.emplace_back(std::move(data), stamp);
    if (records_.size() > max_size_)
    {
      records_.pop_front();
      return true;
    }
    return false;
  }

  bool empty() const
  {
    return records_.empty();
  }

  /// Pops the oldest queued entry. Caller must check empty() first.
  std::pair<nlohmann::json, rclcpp::Time> pop()
  {
    auto record = std::move(records_.front());
    records_.pop_front();
    return record;
  }

private:
  std::size_t max_size_;
  std::deque<std::pair<nlohmann::json, rclcpp::Time>> records_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PENDING_RECORD_QUEUE_HPP_
