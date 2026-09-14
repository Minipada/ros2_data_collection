// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/memory.hpp"

namespace dc_measurements
{

Memory::Memory() : dc_measurements::Measurement()
{
}

Memory::~Memory() = default;

json Memory::collect()
{
  json data_json;
  data_json["used"] = System().memoryUtilization() * 100;
  return data_json;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Memory, dc_core::Measurement)
