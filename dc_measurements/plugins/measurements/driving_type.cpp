// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/driving_type.hpp"

namespace dc_measurements
{

DrivingType::DrivingType() : dc_measurements::Measurement()
{
}

DrivingType::~DrivingType() = default;

void DrivingType::onConfigure()
{
  mode_source_.configure(getNode(), measurement_name_, logger_);
}

void DrivingType::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "driving_type.json");
  }
}

dc_interfaces::msg::StringStamped DrivingType::collect()
{
  json data_json;
  data_json["mode"] = mode_source_.mode();

  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  msg.data = data_json.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::DrivingType, dc_core::Measurement)
