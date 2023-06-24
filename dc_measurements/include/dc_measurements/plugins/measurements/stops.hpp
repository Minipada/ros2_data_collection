
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STOPS_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STOPS_HPP_

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_measurements
{

class Stops : public dc_measurements::Measurement
{
public:
  Stops();
  ~Stops() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  double start_;
  double end_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
  void odomCb(const nav_msgs::msg::Odometry& msg);
  double getFormattedTime();

  std::string odom_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  float speed_threshold_;
  int count_limit_;
  int count_hysteresis_;
  int moving_count_{ 0 };
  bool active_;
  dc_interfaces::msg::StringStamped last_data_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__STOPS_HPP_
