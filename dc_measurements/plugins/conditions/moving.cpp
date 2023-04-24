#include "dc_measurements/plugins/conditions/moving.hpp"

namespace dc_conditions
{

Moving::Moving() : dc_conditions::Condition()
{
}

void Moving::onConfigure()
{
  auto node = getNode();

  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".odom_topic", rclcpp::ParameterValue("/odom"));
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".speed_threshold", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".count_limit", rclcpp::ParameterValue(8));
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".count_hysteresis", rclcpp::ParameterValue(5));
  node->get_parameter(condition_name_ + ".odom_topic", odom_topic_);
  node->get_parameter(condition_name_ + ".speed_threshold", speed_threshold_);
  node->get_parameter(condition_name_ + ".count_limit", count_limit_);
  node->get_parameter(condition_name_ + ".count_hysteresis", count_hysteresis_);

  subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&Moving::odomCb, this, std::placeholders::_1));
}

void Moving::odomCb(nav_msgs::msg::Odometry::SharedPtr msg)
{
  float speed = sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                     msg->twist.twist.linear.y * msg->twist.twist.linear.y);

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
  else if (moving_count_ <= -count_hysteresis_ and active_)
  {
    RCLCPP_DEBUG(logger_, "Stopped, was moving");
    active_ = false;
  }
}

Moving::~Moving() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::Moving, dc_core::Condition)
