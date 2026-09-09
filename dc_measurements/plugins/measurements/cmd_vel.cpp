// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/cmd_vel.hpp"

namespace dc_measurements
{

CmdVel::CmdVel() : dc_measurements::Measurement()
{
}

CmdVel::~CmdVel() = default;

void CmdVel::cmdVelCb(const geometry_msgs::msg::Twist& msg)
{
  std::string yaml_str = geometry_msgs::msg::to_yaml(msg);
  YAML::Node yaml_node = YAML::Load(yaml_str);
  json data_json = dc_util::tojson::detail::yaml2json(yaml_node);
  data_json["computed"] = sqrt(msg.linear.x * msg.linear.x + msg.linear.y * msg.linear.y);
  // Keep-only-the-latest is the shape here, so a displaced value is by design, not a drop to
  // warn about.
  latest_twist_.push({ std::move(data_json), getNode()->get_clock()->now() });
}

void CmdVel::onConfigure()
{
  auto node = getNode();
  cmd_vel_topic_ = dc_util::get_str_type_param(node, measurement_name_, "topic", "/cmd_vel");

  subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10, std::bind(&CmdVel::cmdVelCb, this, std::placeholders::_1));
}

dc_interfaces::msg::StringStamped CmdVel::collect()
{
  dc_interfaces::msg::StringStamped msg;

  const auto twist = latest_twist_.pop();
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
PLUGINLIB_EXPORT_CLASS(dc_measurements::CmdVel, dc_core::Measurement)
