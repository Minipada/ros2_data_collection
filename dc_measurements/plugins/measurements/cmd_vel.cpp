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
  dc_interfaces::msg::StringStamped pub_msg;
  pub_msg.group_key = group_key_;
  pub_msg.data = data_json.dump(-1, ' ', true);
  auto node = getNode();
  pub_msg.header.stamp = node->get_clock()->now();
  last_data_ = pub_msg;
}

void CmdVel::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".topic", rclcpp::ParameterValue("/cmd_vel"));

  node->get_parameter(measurement_name_ + ".topic", cmd_vel_topic_);

  subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10, std::bind(&CmdVel::cmdVelCb, this, std::placeholders::_1));
}

void CmdVel::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "cmd_vel.json");
  }
}

dc_interfaces::msg::StringStamped CmdVel::collect()
{
  dc_interfaces::msg::StringStamped msg = last_data_;
  last_data_ = dc_interfaces::msg::StringStamped();

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::CmdVel, dc_core::Measurement)
