// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/conditions/list_bool_equal.hpp"

namespace dc_conditions
{

ListBoolEqual::ListBoolEqual() : dc_conditions::Condition()
{
}

void ListBoolEqual::onConfigure()
{
  auto node = getNode();
  key_ = dc_util::get_str_type_param(node, condition_name_, "key");
  value_ = dc_util::get_bool_array_type_param(node, condition_name_, "value");
  order_matters_ = dc_util::get_bool_type_param(node, condition_name_, "order_matters", true);
}

bool ListBoolEqual::getState(dc_interfaces::msg::StringStamped msg)
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

  if (!std::all_of(field.begin(), field.end(), [](const json& el) { return el.is_boolean(); }))
  {
    RCLCPP_WARN_STREAM(logger_, "All values are not boolean in key " << key_);
    active_ = false;
    publishActive();
    return active_;
  }

  std::vector<bool> data_bool = field.get<std::vector<bool>>();
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
