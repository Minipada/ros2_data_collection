#include "dc_measurements/plugins/conditions/list_double_equal.hpp"

namespace dc_conditions
{

ListDoubleEqual::ListDoubleEqual() : dc_conditions::Condition()
{
}

void ListDoubleEqual::onConfigure()
{
  auto node = getNode();
  key_ = dc_util::get_str_type_param(node, condition_name_, "key");
  value_ = dc_util::get_double_array_type_param(node, condition_name_, "value");
  order_matters_ = dc_util::get_bool_type_param(node, condition_name_, "order_matters", true);
}

bool ListDoubleEqual::getState(dc_interfaces::msg::StringStamped msg)
{
  json data_json = json::parse(msg.data);

  // json::json_pointer navigates the unflattened data_json directly, unlike the flatten() +
  // "/key" lookup pattern other Condition plugins use: flatten() explodes arrays into indexed
  // keys ("/key/0", "/key/1", ...), so an array-valued "/key" would never be found by that
  // pattern -- exactly the value type this Condition exists to compare.
  json::json_pointer key_ptr(std::string("/") + key_);

  if (!data_json.contains(key_ptr))
  {
    RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not found in msg: " << msg.data);
    active_ = false;
    publishActive();
    return active_;
  }

  const json& field = data_json.at(key_ptr);

  if (field.type() != json::value_t::array)
  {
    RCLCPP_WARN_STREAM(logger_, "Key " << key_ << " not an array");
    active_ = false;
    publishActive();
    return active_;
  }

  if (!std::all_of(field.begin(), field.end(), [](const json& el) { return el.is_number_float(); }))
  {
    RCLCPP_WARN_STREAM(logger_, "All values are not double in key " << key_);
    active_ = false;
    publishActive();
    return active_;
  }

  std::vector<double> data_double = field.get<std::vector<double>>();

  if (order_matters_ && data_double == value_)
  {
    active_ = true;
    publishActive();
    return active_;
  }
  else if (order_matters_ && data_double != value_)
  {
    active_ = false;
    publishActive();
    return active_;
  }

  std::sort(data_double.begin(), data_double.end());
  std::sort(value_.begin(), value_.end());

  if (!order_matters_ && data_double == value_)
  {
    active_ = true;
  }
  else if (!order_matters_ && data_double != value_)
  {
    active_ = false;
  }
  publishActive();
  return active_;
}

ListDoubleEqual::~ListDoubleEqual() = default;

}  // namespace dc_conditions

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_conditions::ListDoubleEqual, dc_core::Condition)
