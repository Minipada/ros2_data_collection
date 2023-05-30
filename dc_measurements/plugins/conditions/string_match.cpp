#include "dc_measurements/plugins/conditions/string_match.hpp"

namespace dc_conditions
{

StringMatch::StringMatch() : dc_conditions::Condition()
{
}

void StringMatch::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".key", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".regex", rclcpp::PARAMETER_STRING);
  node->get_parameter(condition_name_ + ".regex", regex_);
  node->get_parameter(condition_name_ + ".key", key_);
}

bool StringMatch::getState(dc_interfaces::msg::StringStamped msg)
{
  json data_json = json::parse(msg.data);
  json flat_json = data_json.flatten();

  std::string key_w_prefix = std::string("/") + key_;

  if (!flat_json.contains(key_w_prefix))
  {
    RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not found in msg: " << msg.data);
    return false;
  }

  const std::regex base_regex(regex_);

  active_ = std::regex_match(flat_json[key_w_prefix].get<std::string>(), base_regex);
  publishActive();
  return active_;
}

StringMatch::~StringMatch() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::StringMatch, dc_core::Condition)
