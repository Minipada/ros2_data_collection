#include "dc_measurements/plugins/conditions/bool_equal.hpp"

namespace dc_conditions
{

BoolEqual::BoolEqual() : dc_conditions::Condition()
{
}

void BoolEqual::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".key", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".value", rclcpp::PARAMETER_BOOL);
  node->get_parameter(condition_name_ + ".key", key_);
  node->get_parameter(condition_name_ + ".value", value_);
}

bool BoolEqual::getState(dc_interfaces::msg::StringStamped msg)
{
  try
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

    if (flat_json[key_w_prefix].type() != json::value_t::boolean)
    {
      RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not a boolean");
      active_ = false;
      publishActive();
      return active_;
    }

    active_ = flat_json[key_w_prefix] == value_;
    publishActive();
  }
  catch (json::parse_error& e)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON (bool equal): " << msg.data);
  }
  return active_;
}

BoolEqual::~BoolEqual() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::BoolEqual, dc_core::Condition)
