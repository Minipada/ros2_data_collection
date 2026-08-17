// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/conditions/integer_equal.hpp"

namespace dc_conditions
{

IntegerEqual::IntegerEqual() : dc_conditions::Condition()
{
}

void IntegerEqual::onConfigure()
{
  auto node = getNode();
  key_ = dc_util::get_str_type_param(node, condition_name_, "key");
  value_ = dc_util::get_int_type_param(node, condition_name_, "value");
}

bool IntegerEqual::getState(dc_interfaces::msg::StringStamped msg)
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

  // is_number_integer(), not a strict type()==number_integer check: nlohmann::json parses
  // non-negative integer literals (the common case) as number_unsigned, not number_integer, and
  // is_number_integer() is the one that correctly treats both as "an integer".
  if (!flat_json[key_w_prefix].is_number_integer())
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
