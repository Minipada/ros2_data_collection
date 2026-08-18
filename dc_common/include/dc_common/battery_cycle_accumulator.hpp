// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_COMMON_BATTERY_CYCLE_ACCUMULATOR_HPP_
#define DC_COMMON_BATTERY_CYCLE_ACCUMULATOR_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

namespace dc_common
{

/**
 * @brief Whether the pack is on a charger, as the hardware reports it.
 *
 * Values match `sensor_msgs/BatteryState`'s POWER_SUPPLY_STATUS_* constants so the battery
 * Measurement (#361) adapts a message with a cast, but nothing here depends on that package.
 */
enum class PowerSupplyStatus : std::uint8_t
{
  Unknown = 0,
  Charging = 1,
  Discharging = 2,
  NotCharging = 3,
  Full = 4,
};

/**
 * @brief One charging session: when it started, when it ended (unset while still charging), the
 * charge percentage at each boundary, and the depth of the discharge that preceded it.
 *
 * Percentages are optional because hardware leaves the field unset often enough that a session
 * with unknown boundaries is still worth recording.
 */
struct ChargingSession
{
  using TimePoint = std::chrono::system_clock::time_point;
  using Duration = std::chrono::system_clock::duration;

  /// Counts sessions from 1.
  std::uint64_t sequence{ 0 };
  TimePoint started{};
  /// Unset while the session is still open -- see open().
  std::optional<TimePoint> ended;
  std::optional<double> start_percentage;
  std::optional<double> end_percentage;
  /// Percentage points discharged since the previous session ended (since the first sample, for
  /// the first session).
  double preceding_discharge_depth{ 0.0 };

  bool open() const
  {
    return !ended.has_value();
  }

  /// How long the session ran, or has run so far when still open. Clamped at zero.
  Duration duration(TimePoint now) const
  {
    const TimePoint end = ended.value_or(now);
    return end > started ? end - started : Duration::zero();
  }
};

/// What one BatteryCycleAccumulator::update() changed: at most one boundary crossing per sample.
struct BatteryUpdate
{
  /// Set when this sample opened a session -- `ended` unset on it, `preceding_discharge_depth` final.
  std::optional<ChargingSession> started;
  /// Set when this sample closed one, with both boundaries filled in.
  std::optional<ChargingSession> ended;
};

/**
 * @class dc_common::BatteryCycleAccumulator
 * @brief Turns charge-percentage and power-supply-status samples into charging-session boundaries,
 * discharge depth and completed-cycle counts, with no ROS dependency.
 *
 * Sessions are delimited by the power-supply status alone, never by a percentage threshold, so a
 * noisy percentage cannot open and close them repeatedly; an Unknown status carries no information
 * about the charger and so neither opens nor closes one, leaving a momentary dropout as one session
 * rather than two.
 *
 * Cycles come from accumulated discharge, not from counting full charges: every drop in percentage
 * adds to a running total and a completed cycle is kFullCycle percentage points of it, so two half
 * discharges are one cycle rather than two. Percentages are on a 0-100 scale (a
 * `sensor_msgs/BatteryState` reports 0-1, so its adapter scales).
 *
 * Owns no clock -- timestamps arrive as arguments. Consumer is the battery Measurement (#361).
 */
class BatteryCycleAccumulator
{
public:
  using TimePoint = std::chrono::system_clock::time_point;

  /// Percentage points of accumulated discharge that make one completed cycle.
  static constexpr double kFullCycle = 100.0;

  /**
   * @brief Feed one sample. `percentage` is unset when the hardware did not report one; the status
   * is still acted on.
   */
  BatteryUpdate update(std::optional<double> percentage, PowerSupplyStatus status, TimePoint at)
  {
    if (percentage.has_value())
    {
      if (last_percentage_.has_value() && *last_percentage_ > *percentage)
      {
        const double drop = *last_percentage_ - *percentage;
        total_discharge_ += drop;
        discharge_since_session_ += drop;
      }
      last_percentage_ = *percentage;
    }

    BatteryUpdate result;
    if (status == PowerSupplyStatus::Unknown)
    {
      return result;
    }

    const bool on_charger = status == PowerSupplyStatus::Charging;
    if (on_charger && !open_.has_value())
    {
      ChargingSession session;
      session.sequence = ++sequence_;
      session.started = at;
      session.start_percentage = last_percentage_;
      session.preceding_discharge_depth = discharge_since_session_;
      open_ = session;
      result.started = session;
    }
    else if (!on_charger && open_.has_value())
    {
      ChargingSession session = *open_;
      session.ended = at;
      session.end_percentage = last_percentage_;
      open_.reset();
      // Reset here rather than at the start, so the depth a session reports covers exactly the
      // time off the charger; a dip while charging is noise as far as that depth goes.
      discharge_since_session_ = 0.0;
      result.ended = session;
    }
    return result;
  }

  bool charging() const
  {
    return open_.has_value();
  }

  /// The session still charging, or nullopt when the pack is not on a charger.
  const std::optional<ChargingSession>& openSession() const
  {
    return open_;
  }

  /// Most recent percentage reported, or nullopt when none ever was.
  const std::optional<double>& lastPercentage() const
  {
    return last_percentage_;
  }

  /// Percentage points discharged over the accumulator's whole life.
  double accumulatedDischarge() const
  {
    return total_discharge_;
  }

  /// Percentage points discharged since the last session ended -- what the next one will report.
  double dischargeSinceLastSession() const
  {
    return discharge_since_session_;
  }

  std::uint64_t completedCycles() const
  {
    return static_cast<std::uint64_t>(total_discharge_ / kFullCycle);
  }

private:
  std::optional<double> last_percentage_;
  std::optional<ChargingSession> open_;
  double total_discharge_{ 0.0 };
  double discharge_since_session_{ 0.0 };
  std::uint64_t sequence_{ 0 };
};

}  // namespace dc_common

#endif  // DC_COMMON_BATTERY_CYCLE_ACCUMULATOR_HPP_
