// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_COMMON_STATE_TRANSITION_DETECTOR_HPP_
#define DC_COMMON_STATE_TRANSITION_DETECTOR_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <utility>

namespace dc_common
{

/**
 * @brief One state change reported by a StateTransitionDetector.
 *
 * `dwell` is how long `from` was held before `at`; `sequence` counts transitions from 1, so a
 * consumer stamping it onto a Record can tell a dropped Record apart from a state that never
 * changed.
 */
template <typename State>
struct StateTransition
{
  State from;
  State to;
  std::chrono::system_clock::time_point at;
  std::chrono::system_clock::duration dwell;
  std::uint64_t sequence;
};

/**
 * @brief The state currently held, and how long it has been held as of a caller-supplied "now".
 *
 * Returned only by StateTransitionDetector::openInterval(): an interval that has not ended yet is
 * reported through this type and never as a StateTransition, so an unterminated interval cannot be
 * mistaken for a completed one of zero duration.
 */
template <typename State>
struct OpenInterval
{
  State state;
  std::chrono::system_clock::time_point entered;
  std::chrono::system_clock::duration elapsed;
  /// Sequence of the transition that entered this state; 0 when it is the first state ever seen.
  std::uint64_t sequence;
};

/**
 * @class dc_common::StateTransitionDetector
 * @brief Turns a stream of (state, timestamp) samples into transitions, with no ROS dependency.
 *
 * Generic over the state type -- anything equality-comparable and copyable fits, so `driving_type`'s
 * mode strings and a mission's outcome enum both work. update() reports a transition only when the
 * state actually differs from the one held, so a Measurement polling the same state at 10 Hz emits
 * nothing until it changes.
 *
 * Owns no clock: every timestamp arrives as an argument, so a test drives a whole shift in
 * microseconds and shutdown behaviour is reproducible. Consumers are the intervention (#362) and
 * mission Measurements; `driving_type` predates it and is not refactored onto it here.
 */
template <typename State>
class StateTransitionDetector
{
public:
  using TimePoint = std::chrono::system_clock::time_point;
  using Duration = std::chrono::system_clock::duration;

  StateTransitionDetector() = default;

  /// Seed with a state already known to be held since `at`, e.g. one restored across a restart.
  StateTransitionDetector(State initial, TimePoint at) : state_(std::move(initial)), entered_(at)
  {
  }

  /**
   * @brief Feed one sample. Returns the transition it caused, or nullopt when it caused none --
   * either because the state is unchanged or because it is the very first state seen (nothing was
   * left, so there is no previous state or dwell to report).
   */
  std::optional<StateTransition<State>> update(State state, TimePoint at)
  {
    if (!state_.has_value())
    {
      state_ = std::move(state);
      entered_ = at;
      return std::nullopt;
    }

    if (*state_ == state)
    {
      return std::nullopt;
    }

    StateTransition<State> transition{ *state_, state, at, elapsed(entered_, at), ++sequence_ };
    state_ = std::move(state);
    entered_ = at;
    return transition;
  }

  /// The state currently held, or nullopt when no sample has arrived yet.
  const std::optional<State>& state() const
  {
    return state_;
  }

  /// Sequence number of the last transition reported; 0 when none has been.
  std::uint64_t sequence() const
  {
    return sequence_;
  }

  /**
   * @brief The interval still open as of `now`, or nullopt when no sample has arrived yet.
   *
   * What a consumer calls when collection stops, so an interval that never ended is recorded as
   * open with the time it has actually run for.
   */
  std::optional<OpenInterval<State>> openInterval(TimePoint now) const
  {
    if (!state_.has_value())
    {
      return std::nullopt;
    }
    return OpenInterval<State>{ *state_, entered_, elapsed(entered_, now), sequence_ };
  }

private:
  /// Clamped at zero: an out-of-order sample must not report a negative dwell downstream.
  static Duration elapsed(TimePoint from, TimePoint to)
  {
    return to > from ? to - from : Duration::zero();
  }

  std::optional<State> state_;
  TimePoint entered_{};
  std::uint64_t sequence_{ 0 };
};

}  // namespace dc_common

#endif  // DC_COMMON_STATE_TRANSITION_DETECTOR_HPP_
