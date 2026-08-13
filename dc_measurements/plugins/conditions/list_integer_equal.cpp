#include "dc_measurements/plugins/conditions/list_integer_equal.hpp"

namespace dc_conditions
{

ListIntegerEqual::ListIntegerEqual() : dc_conditions::Condition()
{
}

void ListIntegerEqual::onConfigure()
{
  auto node = getNode();
  key_ = dc_util::get_str_type_param(node, condition_name_, "key");
  value_ = dc_util::get_int_array_type_param(node, condition_name_, "value");
  order_matters_ = dc_util::get_bool_type_param(node, condition_name_, "order_matters", true);
}

bool ListIntegerEqual::getState(dc_interfaces::msg::StringStamped msg)
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

  if (!std::all_of(field.begin(), field.end(), [](const json& el) { return el.is_number_integer(); }))
  {
    RCLCPP_WARN_STREAM(logger_, "All values are not integer in key " << key_);
    active_ = false;
    publishActive();
    return active_;
  }

  std::vector<long int> data = field.get<std::vector<long int>>();

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
  else if (!order_matters_ && data != value_)
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
