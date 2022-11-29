#include "speed.hpp"

#include <chrono>
#include <memory>

namespace dc_measurements
{

Speed::Speed() : dc_measurements::Measurement()
{
}

Speed::~Speed() = default;

void Speed::odomCb(nav_msgs::msg::Odometry::SharedPtr msg)
{
  json data_json;
  data_json["linear"]["x"] = msg->twist.twist.linear.x;
  data_json["linear"]["y"] = msg->twist.twist.linear.y;
  data_json["linear"]["z"] = msg->twist.twist.linear.z;
  data_json["angular"]["x"] = msg->twist.twist.angular.x;
  data_json["angular"]["x"] = msg->twist.twist.angular.y;
  data_json["angular"]["x"] = msg->twist.twist.angular.z;
  data_json["computed"] = sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                               msg->twist.twist.linear.y * msg->twist.twist.linear.y);
  dc_interfaces::msg::StringStamped pub_msg;
  pub_msg.group_key = group_key_;
  pub_msg.data = data_json.dump(-1, ' ', true);
  data_json.clear();
  auto node = getNode();
  pub_msg.header.stamp = node->get_clock()->now();
  last_data_ = pub_msg;
}

void Speed::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".odom_topic", rclcpp::ParameterValue("/odom"));

  node->get_parameter(measurement_name_ + ".odom_topic", odom_topic_);

  subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_.c_str(), 10, std::bind(&Speed::odomCb, this, std::placeholders::_1));
}

void Speed::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "speed.json");
  }
}

dc_interfaces::msg::StringStamped Speed::collect()
{
  dc_interfaces::msg::StringStamped msg = last_data_;
  last_data_ = dc_interfaces::msg::StringStamped();
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Speed, dc_core::Measurement)
