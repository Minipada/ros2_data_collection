// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__CMD_VEL_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__CMD_VEL_HPP_

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/source_adapter.hpp"
#include "dc_util/json_utils.hpp"
#include "dc_util/node_utils.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

namespace dc_measurements
{

class CmdVel : public dc_measurements::Measurement
{
public:
  CmdVel();
  ~CmdVel() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  void cmdVelCb(const geometry_msgs::msg::Twist& msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  // The latest decoded Twist, displaced by every newer one and drained one per poll (#502).
  SourceAdapter<std::pair<json, rclcpp::Time>> latest_twist_{ 1 };
  std::string cmd_vel_topic_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__CMD_VEL_HPP_
