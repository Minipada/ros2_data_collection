#include "dc_measurements/plugins/conditions/double_equal.hpp"

namespace dc_conditions
{

DoubleEqual::DoubleEqual() : dc_conditions::Condition()
{
}

void DoubleEqual::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".key", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".value", rclcpp::PARAMETER_DOUBLE);
  node->get_parameter(condition_name_ + ".key", key_);
  node->get_parameter(condition_name_ + ".value", value_);
}

bool DoubleEqual::getState(dc_interfaces::msg::StringStamped msg)
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

    if (flat_json[key_w_prefix].type() != json::value_t::number_float)
    {
      RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not a double");
      active_ = false;
      publishActive();
      return active_;
    }
    return flat_json[key_w_prefix] == value_;
  }
  catch (json::parse_error& e)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON (double equal): " << msg.data);
    return false;
  }
}

DoubleEqual::~DoubleEqual() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::DoubleEqual, dc_core::Condition)
