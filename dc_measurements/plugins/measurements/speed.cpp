// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/speed.hpp"

namespace dc_measurements
{

Speed::Speed() : dc_measurements::Measurement()
{
}

Speed::~Speed() = default;

void Speed::odomCb(const nav_msgs::msg::Odometry& msg)
{
  json data_json;
  data_json["linear"]["x"] = msg.twist.twist.linear.x;
  data_json["linear"]["y"] = msg.twist.twist.linear.y;
  data_json["linear"]["z"] = msg.twist.twist.linear.z;
  data_json["angular"]["x"] = msg.twist.twist.angular.x;
  data_json["angular"]["y"] = msg.twist.twist.angular.y;
  data_json["angular"]["z"] = msg.twist.twist.angular.z;
  data_json["computed"] =
      sqrt(msg.twist.twist.linear.x * msg.twist.twist.linear.x + msg.twist.twist.linear.y * msg.twist.twist.linear.y);
  // Keep-only-the-latest is the shape here, so a displaced value is by design, not a drop to
  // warn about.
  latest_odom_.push({ std::move(data_json), getNode()->get_clock()->now() });
}

void Speed::onConfigure()
{
  auto node = getNode();
  odom_topic_ = dc_util::get_str_type_param(node, measurement_name_, "odom_topic", "/odom");

  subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&Speed::odomCb, this, std::placeholders::_1));
}

dc_interfaces::msg::StringStamped Speed::collect()
{
  dc_interfaces::msg::StringStamped msg;

  const auto twist = latest_odom_.pop();
  if (!twist)
  {
    return msg;
  }
  msg.group_key = group_key_;
  msg.header.stamp = twist->second;
  msg.data = twist->first.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Speed, dc_core::Measurement)
