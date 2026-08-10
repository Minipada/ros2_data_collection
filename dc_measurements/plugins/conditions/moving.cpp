#include "dc_measurements/plugins/conditions/moving.hpp"

namespace dc_conditions
{

Moving::Moving() : dc_conditions::Condition()
{
}

void Moving::onConfigure()
{
  auto node = getNode();

  odom_topic_ = dc_util::get_str_type_param(node, condition_name_, "odom_topic", "/odom");
  speed_threshold_ = dc_util::get_double_type_param(node, condition_name_, "speed_threshold", 0.2);
  count_limit_ = dc_util::get_int_type_param(node, condition_name_, "count_limit", 8);
  count_hysteresis_ = dc_util::get_int_type_param(node, condition_name_, "count_hysteresis", 5);

  subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&Moving::odomCb, this, std::placeholders::_1));
}

void Moving::odomCb(const nav_msgs::msg::Odometry& msg)
{
  float speed =
      sqrt(msg.twist.twist.linear.x * msg.twist.twist.linear.x + msg.twist.twist.linear.y * msg.twist.twist.linear.y);

  if (speed > speed_threshold_)
  {
    moving_count_ = moving_count_ + 1;
  }
  else
  {
    moving_count_ = moving_count_ - 1;
  }

  if (moving_count_ > count_limit_)
  {
    moving_count_ = count_limit_;
  }
  else if (moving_count_ < -count_limit_)
  {
    moving_count_ = -count_limit_;
  }

  if (moving_count_ >= count_hysteresis_ && !active_)
  {
    RCLCPP_DEBUG(logger_, "Moving, was stopped");
    active_ = true;
  }
  else if (moving_count_ <= -count_hysteresis_ && active_)
  {
    RCLCPP_DEBUG(logger_, "Stopped, was moving");
    active_ = false;
  }
}

Moving::~Moving() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::Moving, dc_core::Condition)
