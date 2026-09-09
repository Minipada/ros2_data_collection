// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/string_stamped.hpp"

namespace dc_measurements
{

StringStamped::StringStamped() : dc_measurements::Measurement()
{
}

StringStamped::~StringStamped() = default;

void StringStamped::onConfigure()
{
  auto node = getNode();
  topic_ = dc_util::get_str_type_param(node, measurement_name_, "topic");
  timer_based_ = dc_util::get_bool_type_param(node, measurement_name_, "timer_based", true);

  subscription_ = node->create_subscription<dc_interfaces::msg::StringStamped>(
      topic_, 10, std::bind(&StringStamped::dataCb, this, std::placeholders::_1));
}

void StringStamped::dataCb(const dc_interfaces::msg::StringStamped& msg)
{
  // Keep-only-the-latest is the shape here, so a displaced Record is by design, not a drop to
  // warn about.
  latest_record_.push(msg);
  if (!timer_based_)
  {
    publishFromMsg(msg);
  }
}

dc_interfaces::msg::StringStamped StringStamped::collect()
{
  const auto cached = latest_record_.pop();
  return cached ? *cached : dc_interfaces::msg::StringStamped{};
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::StringStamped, dc_core::Measurement)
