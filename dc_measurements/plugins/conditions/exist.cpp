#include "dc_measurements/plugins/conditions/exist.hpp"

namespace dc_conditions
{

Exist::Exist() : dc_conditions::Condition()
{
}

void Exist::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".key", rclcpp::PARAMETER_STRING);
  node->get_parameter(condition_name_ + ".key", key_);
}

bool Exist::getState(dc_interfaces::msg::StringStamped msg)
{
  json data_json = json::parse(msg.data);
  json flat_json = data_json.flatten();

  std::string key_w_prefix = std::string("/") + key_;

  return flat_json.contains(key_w_prefix);
}

Exist::~Exist() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::Exist, dc_core::Condition)
