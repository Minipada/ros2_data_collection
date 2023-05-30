#include "dc_measurements/plugins/conditions/double_inferior.hpp"

namespace dc_conditions
{

DoubleInferior::DoubleInferior() : dc_conditions::Condition()
{
}

void DoubleInferior::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".key", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".value", rclcpp::PARAMETER_DOUBLE);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".include_value", rclcpp::ParameterValue(true));
  node->get_parameter(condition_name_ + ".key", key_);
  node->get_parameter(condition_name_ + ".value", value_);
  node->get_parameter(condition_name_ + ".include_value", include_value_);
}

bool DoubleInferior::getState(dc_interfaces::msg::StringStamped msg)
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

    if (include_value_)
    {
      active_ = flat_json[key_w_prefix] <= value_;
      publishActive();
      return active_;
    }
    else
    {
      active_ = flat_json[key_w_prefix] < value_;
      publishActive();
      return active_;
    }
  }
  catch (json::parse_error& e)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing JSON (double inferior): " << msg.data);
    return false;
  }
}

DoubleInferior::~DoubleInferior() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::DoubleInferior, dc_core::Condition)
