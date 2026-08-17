#ifndef DC_COMMON_RECORD_RING_BUFFER_HPP_
#define DC_COMMON_RECORD_RING_BUFFER_HPP_

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace dc_common
{

/**
 * @brief One stamped Record held by a RecordRingBuffer: its JSON payload plus the timestamp it
 * was pushed with.
 */
struct StampedRecord
{
  std::string json;
  std::chrono::system_clock::time_point stamp;
};

/**
 * @class dc_common::RecordRingBuffer
 * @brief In-memory bounded time-window buffer of stamped Record JSON strings, with no ROS
 * dependency.
 *
 * push() inserts a Record in ascending-stamp order (so out-of-order pushes still leave window()
 * in time order) but never evicts on its own; evict() drops entries older than the configured
 * window relative to a caller-supplied "now". Keeping the two separate means the window is always
 * relative to wall-clock "now", not to whenever the last push happened -- important for a buffer
 * that may sit idle between pushes yet still needs to age out.
 *
 * Part of pre-event circular-buffer capture (#282): a Trigger plugin keeps pushing Records into
 * one of these as they're produced and, on trigger, reads window() to get "the last N seconds of
 * data" regardless of when the trigger itself fires.
 */
class RecordRingBuffer
{
public:
  /**
   * @param window Entries older than this age (relative to the `now` passed to evict()) are
   * eligible for eviction. Age exactly equal to `window` is kept -- see evict().
   */
  explicit RecordRingBuffer(std::chrono::system_clock::duration window) : window_(window)
  {
  }

  /**
   * @brief Insert one stamped Record, maintaining ascending-stamp order.
   *
   * Does not evict; call evict() to enforce the configured time window.
   */
  void push(std::string json, std::chrono::system_clock::time_point stamp)
  {
    const auto pos = std::upper_bound(entries_.begin(), entries_.end(), stamp,
                                      [](const std::chrono::system_clock::time_point& s, const StampedRecord& e) {
                                        return s < e.stamp;
                                      });
    entries_.insert(pos, StampedRecord{ std::move(json), stamp });
  }

  /**
   * @brief Drop every entry older than the configured window relative to `now`. An entry exactly
   * `window` old is kept (inclusive boundary).
   */
  void evict(std::chrono::system_clock::time_point now)
  {
    while (!entries_.empty() && (now - entries_.front().stamp) > window_)
    {
      entries_.pop_front();
    }
  }

  /**
   * @brief Drop every entry regardless of age.
   *
   * For the caller that has just consumed the whole window and must not see those entries again --
   * dc_measurements::IncidentReleaser after releasing one incident's pre-roll (#288).
   */
  void clear()
  {
    entries_.clear();
  }

  /// Current contents, oldest first. Does not evict -- call evict() first if that's wanted.
  std::vector<StampedRecord> window() const
  {
    return std::vector<StampedRecord>(entries_.begin(), entries_.end());
  }

  std::size_t size() const
  {
    return entries_.size();
  }

  bool empty() const
  {
    return entries_.empty();
  }

private:
  std::chrono::system_clock::duration window_;
  std::deque<StampedRecord> entries_;
};

}  // namespace dc_common

#endif  // DC_COMMON_RECORD_RING_BUFFER_HPP_
