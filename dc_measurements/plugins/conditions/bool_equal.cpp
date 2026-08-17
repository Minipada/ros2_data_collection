// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/conditions/bool_equal.hpp"

namespace dc_conditions
{

BoolEqual::BoolEqual() : dc_conditions::Condition()
{
}

void BoolEqual::onConfigure()
{
  auto node = getNode();
  key_ = dc_util::get_str_type_param(node, condition_name_, "key");
  // NOTE: `value_` is `double` (dc_measurements/plugins/conditions/bool_equal.hpp) despite the
  // parameter being declared PARAMETER_BOOL -- a pre-existing type mismatch, left as its
  // original direct nav2_util call rather than routed through dc_util::get_bool_type_param,
  // which would change what type get_parameter() reads and so risk changing behavior. Not in
  // scope for #178; worth its own follow-up.
  nav2_util::declare_parameter_if_not_declared(node, condition_name_ + ".value", rclcpp::PARAMETER_BOOL);
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
