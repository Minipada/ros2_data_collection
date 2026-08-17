#ifndef DC_MEASUREMENTS__INCIDENT_RELEASER_HPP_
#define DC_MEASUREMENTS__INCIDENT_RELEASER_HPP_

#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <utility>

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
 *   incident_id the FlushEvent carried. Release is currently immediate, so this state is entered
 *   and left within onFlush(); it exists as a distinct state because rate-limited release (the
 *   `max release rate` knob of #282) drains the window across several tick()s instead.
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
   * @param publish Called for every released and every post-roll Record.
   */
  IncidentReleaser(const Duration& buffer_window, const Duration& post_roll, const Duration& cooldown, PublishFn publish)
    : post_roll_(post_roll), cooldown_(cooldown), publish_(std::move(publish)), buffer_(buffer_window)
  {
  }

  /**
   * @brief Hand one freshly collected sample to the machine.
   *
   * Buffered while Buffering or in Cooldown, published live (tagged with the current incident_id)
   * during PostRoll. Drives time-based transitions first, so a sample collected after the
   * post-roll deadline is buffered rather than published even if nothing else called tick().
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
    for (const auto& entry : buffer_.window())
    {
      publish_(entry.json, entry.stamp, incident_id_);
    }
    // Released entries are gone for good: the next incident gets its own window, not this one
    // replayed a second time.
    buffer_.clear();

    if (post_roll_ > Duration::zero())
    {
      state_ = IncidentState::PostRoll;
      phase_deadline_ = now + post_roll_;
      return;
    }
    enterCooldown(now);
  }

  /**
   * @brief Advance the time-based transitions (post-roll expiry, cooldown expiry) to `now`.
   *
   * Safe and cheap to call at any rate; offer() and onFlush() call it themselves, so a
   * Measurement driving both from its polling timer needs no separate tick.
   */
  void tick(const TimePoint& now)
  {
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

private:
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
  PublishFn publish_;

  dc_common::RecordRingBuffer buffer_;
  IncidentState state_{ IncidentState::Buffering };
  std::string incident_id_;
  // End of the phase in progress; only meaningful in PostRoll and Cooldown.
  TimePoint phase_deadline_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__INCIDENT_RELEASER_HPP_
