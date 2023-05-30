#include "dc_measurements/plugins/conditions/list_bool_equal.hpp"

namespace dc_conditions
{

ListBoolEqual::ListBoolEqual() : dc_conditions::Condition()
{
}

void ListBoolEqual::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".key", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".value", rclcpp::PARAMETER_BOOL_ARRAY);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".order_matters", rclcpp::ParameterValue(true));
  node->get_parameter(condition_name_ + ".key", key_);
  node->get_parameter(condition_name_ + ".value", value_);
  node->get_parameter(condition_name_ + ".order_matters", order_matters_);
}

bool ListBoolEqual::getState(dc_interfaces::msg::StringStamped msg)
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

  if (flat_json[key_w_prefix].type() != json::value_t::array)
  {
    RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not an array");
    active_ = false;
    publishActive();
    return active_;
  }

  if (!std::all_of(flat_json[key_w_prefix].begin(), flat_json[key_w_prefix].end(),
                   [](const json& el) { return el.is_boolean(); }))
  {
    RCLCPP_WARN_STREAM(logger_, "All values are not boolean in key " << key_);
    active_ = false;
    publishActive();
    return active_;
  }

  std::vector<bool> data_bool = flat_json[key_w_prefix].get<std::vector<bool>>();
  std::vector<int> data_int(data_bool.begin(), data_bool.end());
  std::vector<int> value_int(value_.begin(), value_.end());

  if (order_matters_ && data_bool == value_)
  {
    active_ = true;
    publishActive();
    return active_;
  }
  else if (order_matters_ && data_bool != value_)
  {
    active_ = false;
    publishActive();
    return active_;
  }

  std::sort(data_int.begin(), data_int.end());
  std::sort(value_int.begin(), value_int.end());

  if (!order_matters_ && data_int == value_int)
  {
    active_ = true;
  }
  else if (!order_matters_ && data_int != value_int)
  {
    active_ = false;
  }
  publishActive();
  return active_;
}

ListBoolEqual::~ListBoolEqual() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::ListBoolEqual, dc_core::Condition)
