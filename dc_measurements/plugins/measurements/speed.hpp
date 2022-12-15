
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SPEED_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SPEED_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace dc_measurements
{

class Speed : public dc_measurements::Measurement
{
public:
  Speed();
  ~Speed();
  dc_interfaces::msg::StringStamped collect() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
  void odomCb(nav_msgs::msg::Odometry::SharedPtr msg);

  std::string odom_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  dc_interfaces::msg::StringStamped last_data_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SPEED_HPP_