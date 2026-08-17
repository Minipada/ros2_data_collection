#ifndef DC_MEASUREMENTS__INCIDENT_RELEASER_HPP_
#define DC_MEASUREMENTS__INCIDENT_RELEASER_HPP_

#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "dc_common/record_ring_buffer.hpp"

namespace dc_measurements
{

/// The four phases of one incident-capture cycle, see IncidentReleaser.
enum class IncidentState : int8_t
{
  Buffering = 0,
  Flushing = 1,
  PostRoll = 2,
  Cooldown = 3,
};

inline const char* toString(const IncidentState& state)
{
  switch (state)
  {
    case IncidentState::Buffering:
      return "Buffering";
    case IncidentState::Flushing:
      return "Flushing";
    case IncidentState::PostRoll:
      return "PostRoll";
    case IncidentState::Cooldown:
      return "Cooldown";
  }
  return "Unknown";
}

/// Seconds as a double (how the ROS parameters express durations) to a system_clock duration.
inline std::chrono::system_clock::duration durationFromSeconds(const double& seconds)
{
  return std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::duration<double>(seconds));
}

/**
 * @class dc_measurements::IncidentReleaser
 * @brief Per-Measurement incident-capture state machine over a RecordRingBuffer, with no ROS
 * dependency (#288).
 *
 * One cycle:
 *
 *   Buffering --onFlush--> Flushing --> PostRoll --> Cooldown --> Buffering
 *
 * - **Buffering** (the initial state): every offered sample goes into the ring buffer, nothing is
 *   published. This is the only state in which a FlushEvent is acted upon (see isArmed()).
 * - **Flushing**: the buffered window is released, oldest first, each Record tagged with the
 *   incident_id the FlushEvent carried. With no `max_flush_rate_hz` configured the whole window
 *   goes out in one burst, so the state is entered and left within onFlush(); with a rate
 *   configured the window is drained across several tick()s instead (#289), and samples collected
 *   meanwhile go back into the ring buffer rather than being published.
 * - **PostRoll**: samples are published live for `post_roll` after the flush, still tagged with
 *   the same incident_id, so the aftermath of the incident is captured too. A zero `post_roll`
 *   (the default) skips the phase entirely -- pre-roll only.
 * - **Cooldown**: further FlushEvents are ignored for `cooldown` after post-roll ends, so a
 *   flapping Trigger can't produce a flood of overlapping incidents. Samples are buffered again
 *   during this phase, so the next incident still gets a full pre-roll window rather than one
 *   with a `post_roll + cooldown`-sized hole at its start.
 *
 * After cooldown the machine re-arms itself back to Buffering with no manual intervention -- a
 * second incident is captured like the first.
 *
 * Time is supplied by the caller on every entry point rather than read from a clock, and Records
 * go out through a caller-supplied callback rather than a publisher, which is what lets the whole
 * cycle be tested with a fake clock and no ROS node.
 */
class IncidentReleaser
{
public:
  using Duration = std::chrono::system_clock::duration;
  using TimePoint = std::chrono::system_clock::time_point;

  /**
   * @brief Emit one Record: its (already enriched) JSON payload, the timestamp it was collected
   * at, and the incident_id of the cycle releasing it.
   */
  using PublishFn =
      std::function<void(const std::string& record_json, const TimePoint& stamp, const std::string& incident_id)>;

  /**
   * @param buffer_window How much history to keep buffered while armed (`buffer_duration_sec`).
   * @param post_roll How long to keep publishing live after a flush (`post_roll_duration_sec`);
   * zero or less means pre-roll only.
   * @param cooldown How long to ignore FlushEvents once post-roll ends (`cooldown_sec`); zero or
   * less re-arms immediately.
   * @param max_flush_rate_hz Ceiling on how fast the buffered window is emitted during Flushing
   * (`max_flush_rate_hz`); zero or less releases the whole window in one burst.
   * @param publish Called for every released and every post-roll Record.
   */
  IncidentReleaser(const Duration& buffer_window, const Duration& post_roll, const Duration& cooldown,
                   const double& max_flush_rate_hz, PublishFn publish)
    : post_roll_(post_roll)
    , cooldown_(cooldown)
    , release_interval_(max_flush_rate_hz > 0.0 ? durationFromSeconds(1.0 / max_flush_rate_hz) : Duration::zero())
    , publish_(std::move(publish))
    , buffer_(buffer_window)
  {
  }

  /**
   * @brief Hand one freshly collected sample to the machine.
   *
   * Buffered while Buffering, Flushing or Cooldown, published live (tagged with the current
   * incident_id) during PostRoll. Drives time-based transitions first, so a sample collected after
   * the post-roll deadline is buffered rather than published even if nothing else called tick().
   */
  void offer(const std::string& record_json, const TimePoint& now)
  {
    tick(now);

    if (state_ == IncidentState::PostRoll)
    {
      publish_(record_json, now, incident_id_);
      return;
    }

    buffer_.push(record_json, now);
    buffer_.evict(now);
  }

  /**
   * @brief A FlushEvent arrived: release the buffered window under `incident_id` and start the
   * post-roll/cooldown phases.
   *
   * Ignored unless armed -- an event arriving during flushing, post-roll, or cooldown belongs to
   * the incident already being captured (or to a flapping Trigger) and starts no new cycle.
   */
  void onFlush(const std::string& incident_id, const TimePoint& now)
  {
    tick(now);

    if (!isArmed())
    {
      return;
    }

    state_ = IncidentState::Flushing;
    incident_id_ = incident_id;

    buffer_.evict(now);
    // Released entries are gone from the buffer for good: the next incident gets its own window,
    // not this one replayed a second time. They live in pending_ until the drain emits them.
    pending_ = buffer_.window();
    pending_index_ = 0;
    buffer_.clear();

    next_release_ = now;
    drainRelease(now);
  }

  /**
   * @brief Advance the time-based transitions (rate-limited release, post-roll expiry, cooldown
   * expiry) to `now`.
   *
   * Safe and cheap to call at any rate; offer() and onFlush() call it themselves, so a
   * Measurement driving both from its polling timer needs no separate tick -- though with
   * `max_flush_rate_hz` set above the polling rate a dedicated timer keeps the drain smooth.
   */
  void tick(const TimePoint& now)
  {
    if (state_ == IncidentState::Flushing)
    {
      drainRelease(now);
    }
    if (state_ == IncidentState::PostRoll && now >= phase_deadline_)
    {
      // Cooldown runs from when post-roll actually ended, not from whenever this tick noticed it,
      // so the total suppression is `post_roll + cooldown` however coarsely tick() is called.
      enterCooldown(phase_deadline_);
    }
    if (state_ == IncidentState::Cooldown && now >= phase_deadline_)
    {
      enterBuffering();
    }
  }

  /// Whether a FlushEvent would start a new capture cycle right now.
  bool isArmed() const
  {
    return state_ == IncidentState::Buffering;
  }

  IncidentState state() const
  {
    return state_;
  }

  /// The incident_id of the cycle in progress; empty while armed.
  const std::string& incidentId() const
  {
    return incident_id_;
  }

  /// How many samples are currently buffered, awaiting a flush.
  std::size_t bufferedCount() const
  {
    return buffer_.size();
  }

  /// How many released Records are still queued behind the flush rate limit; 0 unless Flushing.
  std::size_t pendingReleaseCount() const
  {
    return pending_.size() - pending_index_;
  }

private:
  /**
   * @brief Emit as many of the still-pending Records as `max_flush_rate_hz` allows by `now`, and
   * leave Flushing once the whole window is out.
   *
   * The cap is on the emission rate over time, not on how many Records one call may emit: when
   * tick() is called more coarsely than the release interval the drain catches up by emitting
   * everything that came due since the last call, and never more than that. Without the catch-up a
   * Measurement whose polling timer is slower than `max_flush_rate_hz` could never reach the
   * configured rate, and the release would take as long as the buffer window itself.
   */
  void drainRelease(const TimePoint& now)
  {
    while (pending_index_ < pending_.size())
    {
      if (release_interval_ > Duration::zero() && now < next_release_)
      {
        // Still Flushing: the next tick() picks the drain up where this one left off.
        return;
      }
      const auto& entry = pending_[pending_index_++];
      publish_(entry.json, entry.stamp, incident_id_);
      next_release_ += release_interval_;
    }

    pending_.clear();
    pending_index_ = 0;
    // Post-roll runs from the end of the release, not from the FlushEvent, so a rate-limited drain
    // doesn't eat into the aftermath window it is supposed to capture.
    if (post_roll_ > Duration::zero())
    {
      state_ = IncidentState::PostRoll;
      phase_deadline_ = now + post_roll_;
      return;
    }
    enterCooldown(now);
  }

  // `from` is taken by value on purpose: tick() passes phase_deadline_ itself, and the assignment
  // below would otherwise change the value `from` refers to mid-function.
  void enterCooldown(TimePoint from)
  {
    state_ = IncidentState::Cooldown;
    phase_deadline_ = from + cooldown_;
    if (from >= phase_deadline_)
    {
      // Nothing to suppress: re-arm without waiting for another tick().
      enterBuffering();
    }
  }

  void enterBuffering()
  {
    state_ = IncidentState::Buffering;
    incident_id_.clear();
  }

  Duration post_roll_;
  Duration cooldown_;
  // Minimum spacing between two released Records; zero means "no limit, release in one burst".
  Duration release_interval_;
  PublishFn publish_;

  dc_common::RecordRingBuffer buffer_;
  IncidentState state_{ IncidentState::Buffering };
  std::string incident_id_;
  // End of the phase in progress; only meaningful in PostRoll and Cooldown.
  TimePoint phase_deadline_;
  // The window being released, oldest first, and how far the drain has got through it. An index
  // rather than a pop_front so the released Records are not shuffled down one by one.
  std::vector<dc_common::StampedRecord> pending_;
  std::size_t pending_index_{ 0 };
  // When the next pending Record may go out; only meaningful in Flushing.
  TimePoint next_release_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__INCIDENT_RELEASER_HPP_
