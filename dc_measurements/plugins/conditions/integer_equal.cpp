#include "dc_measurements/plugins/conditions/integer_equal.hpp"

namespace dc_conditions
{

IntegerEqual::IntegerEqual() : dc_conditions::Condition()
{
}

void IntegerEqual::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".key", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".value", rclcpp::PARAMETER_INTEGER);
  node->get_parameter(condition_name_ + ".key", key_);
  node->get_parameter(condition_name_ + ".value", value_);
}

bool IntegerEqual::getState(dc_interfaces::msg::StringStamped msg)
{
  json data_json = json::parse(msg.data);
  json flat_json = data_json.flatten();

  std::string key_w_prefix = std::string("/") + key_;

  if (!flat_json.contains(key_w_prefix))
  {
    RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not found");
    active_ = false;
    publishActive();
    return active_;
  }

  if (flat_json[key_w_prefix].type() != json::value_t::number_integer)
  {
    RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not an integer");
    active_ = false;
    publishActive();
    return active_;
  }

  active_ = flat_json[key_w_prefix] == value_;
  publishActive();
  return active_;
}

IntegerEqual::~IntegerEqual() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::IntegerEqual, dc_core::Condition)
