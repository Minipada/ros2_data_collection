#ifndef DC_MEASUREMENTS__PLUGINS__CONDITION__MOVING_HPP_
#define DC_MEASUREMENTS__PLUGINS__CONDITION__MOVING_HPP_

#include <math.h>

#include "dc_core/condition.hpp"
#include "dc_measurements/condition.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace dc_conditions
{

class Moving : public dc_conditions::Condition
{
public:
  Moving();
  ~Moving() override;

private:
  std::string odom_topic_;
  void odomCb(nav_msgs::msg::Odometry::SharedPtr msg);
  void onConfigure() override;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

  float speed_threshold_;
  int count_limit_;
  int count_hysteresis_;
  int moving_count_{ 0 };
};

}  // namespace dc_conditions

#endif  // DC_MEASUREMENTS__PLUGINS__CONDITION__MOVING_HPP_
