// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/dummy.hpp"

namespace dc_measurements
{

Dummy::Dummy() : dc_measurements::Measurement()
{
}

Dummy::~Dummy() = default;

void Dummy::onConfigure()
{
  auto node = getNode();
  record_ = dc_util::get_str_type_param(node, measurement_name_, "record", "{\"message\":\"Hello from ROS 2 DC\"}");
}

json Dummy::collect()
{
  try
  {
    return json::parse(record_);
  }
  catch (json::parse_error& ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Could not parse record as JSON: " << record_);
  }
  // Unparsable: report nothing, exactly as the old empty Record did.
  return json{};
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Dummy, dc_core::Measurement)
