#include "dc_measurements/plugins/conditions/list_integer_equal.hpp"

namespace dc_conditions
{

ListIntegerEqual::ListIntegerEqual() : dc_conditions::Condition()
{
}

void ListIntegerEqual::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".key", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".value", rclcpp::PARAMETER_INTEGER_ARRAY);
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".order_matters", rclcpp::ParameterValue(true));
  node->get_parameter(condition_name_ + ".key", key_);
  node->get_parameter(condition_name_ + ".value", value_);
  node->get_parameter(condition_name_ + ".order_matters", order_matters_);
}

bool ListIntegerEqual::getState(dc_interfaces::msg::StringStamped msg)
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
                   [](const json& el) { return el.is_number_integer(); }))
  {
    RCLCPP_WARN_STREAM(logger_, "All values are not integer in key " << key_);
    active_ = false;
    publishActive();
    return active_;
  }

  std::vector<long int> data = flat_json[key_w_prefix].get<std::vector<long int>>();

  if (order_matters_ && data == value_)
  {
    active_ = true;
    publishActive();
    return active_;
  }
  else if (order_matters_ && data != value_)
  {
    active_ = false;
    publishActive();
    return active_;
  }

  std::sort(data.begin(), data.end());
  std::sort(value_.begin(), value_.end());

  if (!order_matters_ && data == value_)
  {
    active_ = true;
  }
  else if (!order_matters_ && data == value_)
  {
    active_ = false;
  }
  publishActive();
  return active_;
}

ListIntegerEqual::~ListIntegerEqual() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::ListIntegerEqual, dc_core::Condition)
