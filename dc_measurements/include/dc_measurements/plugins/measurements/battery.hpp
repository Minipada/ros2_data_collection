// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__BATTERY_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__BATTERY_HPP_

#include <string>
#include <utility>

#include "dc_common/battery_cycle_accumulator.hpp"
#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/source_adapter.hpp"
#include "dc_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace dc_measurements
{

class Battery : public dc_measurements::Measurement
{
public:
  Battery();
  ~Battery() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  void batteryStateCb(const sensor_msgs::msg::BatteryState& msg);
  json sampleRecord(const sensor_msgs::msg::BatteryState& msg) const;

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
  std::string battery_topic_;
  double percentage_scale_{ 100.0 };

  dc_common::BatteryCycleAccumulator accumulator_;
  // The decoded sample, re-read on every poll until a newer BatteryState lands -- a sample
  // source, not an event one (#502).
  SourceAdapter<json> sample_{ 1 };
  // Session boundaries wait here for a poll to carry them out, one Record per poll, so they
  // travel the same publish path (Conditions, buffering, Group) as every other Record.
  SourceAdapter<std::pair<json, rclcpp::Time>> pending_events_{ 64 };

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__BATTERY_HPP_
