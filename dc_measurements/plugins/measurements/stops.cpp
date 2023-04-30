#include "dc_measurements/plugins/measurements/stops.hpp"

namespace dc_measurements
{

Stops::Stops() : dc_measurements::Measurement()
{
}

Stops::~Stops() = default;

void Stops::onConfigure()
{
  auto node = getNode();

  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".odom_topic", rclcpp::ParameterValue("/odom"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".speed_threshold",
                                               rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".count_limit", rclcpp::ParameterValue(8));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".count_hysteresis", rclcpp::ParameterValue(5));
  node->get_parameter(measurement_name_ + ".odom_topic", odom_topic_);
  node->get_parameter(measurement_name_ + ".speed_threshold", speed_threshold_);
  node->get_parameter(measurement_name_ + ".count_limit", count_limit_);
  node->get_parameter(measurement_name_ + ".count_hysteresis", count_hysteresis_);

  start_ = getFormattedTime();
  subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&Stops::odomCb, this, std::placeholders::_1));
}

double Stops::getFormattedTime()
{
  auto node = node_.lock();
  auto now_nsecs = node->get_clock()->now().nanoseconds();
  int seconds = now_nsecs / 1e9;
  auto nseconds = int(now_nsecs - seconds * 1e9);
  return std::stod(std::to_string(seconds) + std::string(".") + std::to_string(nseconds));
}

void Stops::odomCb(const nav_msgs::msg::Odometry& msg)
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

  auto formatted_time = getFormattedTime();
  if (moving_count_ >= count_hysteresis_ && !active_)
  {
    RCLCPP_DEBUG(logger_, "Moving, was stopped");
    active_ = true;
    end_ = formatted_time;

    json data_json;
    data_json["start"] = start_;
    data_json["end"] = end_;
    data_json["duration"] = end_ - start_;

    dc_interfaces::msg::StringStamped pub_msg;
    pub_msg.group_key = group_key_;
    pub_msg.data = data_json.dump(-1, ' ', true);
    data_json.clear();
    auto node = getNode();
    pub_msg.header.stamp = node->get_clock()->now();
    last_data_ = pub_msg;
  }
  else if (moving_count_ <= -count_hysteresis_ && active_)
  {
    active_ = false;
    RCLCPP_DEBUG(logger_, "Stopped, was moving");
    start_ = formatted_time;
  }
}

void Stops::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "stops.json");
  }
}

dc_interfaces::msg::StringStamped Stops::collect()
{
  dc_interfaces::msg::StringStamped msg = last_data_;
  last_data_ = dc_interfaces::msg::StringStamped();
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Stops, dc_core::Measurement)
