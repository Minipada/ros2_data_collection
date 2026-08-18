// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__BATTERY_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__BATTERY_HPP_

#include <deque>
#include <mutex>
#include <string>
#include <utility>

#include "dc_common/battery_cycle_accumulator.hpp"
#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
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
  json sampleRecord() const;

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
  std::string battery_topic_;
  double percentage_scale_{ 100.0 };

  // The BatteryState callback and the polling timer run in different callback groups under a
  // multi-threaded executor, so everything they share is guarded.
  mutable std::mutex mutex_;
  dc_common::BatteryCycleAccumulator accumulator_;
  sensor_msgs::msg::BatteryState last_msg_;
  bool has_sample_{ false };
  // Session boundaries wait here for a poll to carry them out, one Record per poll, so they
  // travel the same publish path (Conditions, buffering, Group) as every other Record.
  std::deque<std::pair<json, rclcpp::Time>> pending_events_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__BATTERY_HPP_
