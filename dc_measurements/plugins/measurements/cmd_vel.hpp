
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__CMD_VEL_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__CMD_VEL_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

namespace dc_measurements
{

class CmdVel : public dc_measurements::Measurement
{
public:
  CmdVel();
  ~CmdVel();
  dc_interfaces::msg::StringStamped collect() override;

private:
  void cmdVelCb(geometry_msgs::msg::Twist::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  dc_interfaces::msg::StringStamped last_data_;
  std::string cmd_vel_topic_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__CMD_VEL_HPP_
