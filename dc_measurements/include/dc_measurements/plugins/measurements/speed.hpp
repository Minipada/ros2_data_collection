// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SPEED_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SPEED_HPP_

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/source_adapter.hpp"
#include "dc_util/node_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace dc_measurements
{

class Speed : public dc_measurements::Measurement
{
public:
  Speed();
  ~Speed() override;
  dc_interfaces::msg::StringStamped collect() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void odomCb(const nav_msgs::msg::Odometry& msg);

  std::string odom_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  // The latest decoded twist, displaced by every newer one and drained one per poll (#502).
  SourceAdapter<std::pair<json, rclcpp::Time>> latest_odom_{ 1 };
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SPEED_HPP_
