#include "dc_measurements/plugins/conditions/integer_inferior.hpp"

namespace dc_conditions
{

IntegerInferior::IntegerInferior() : dc_conditions::Condition()
{
}

void IntegerInferior::onConfigure()
{
  auto node = getNode();
  key_ = dc_util::get_str_type_param(node, condition_name_, "key");
  value_ = dc_util::get_int_type_param(node, condition_name_, "value");
  include_value_ = dc_util::get_bool_type_param(node, condition_name_, "include_value", true);
}

bool IntegerInferior::getState(dc_interfaces::msg::StringStamped msg)
{
  json data_json = json::parse(msg.data);
  json flat_json = data_json.flatten();

  std::string key_w_prefix = std::string("/") + key_;

  if (!flat_json.contains(key_w_prefix))
  {
    RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not found in msg: " << msg.data);
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

  if (include_value_)
  {
    active_ = flat_json[key_w_prefix] <= value_;
  }
  else
  {
    active_ = flat_json[key_w_prefix] < value_;
  }
  publishActive();
  return active_;
}

IntegerInferior::~IntegerInferior() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::IntegerInferior, dc_core::Condition)
