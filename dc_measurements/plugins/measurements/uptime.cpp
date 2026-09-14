// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/uptime.hpp"

namespace dc_measurements
{
namespace lp = LinuxParser;

Uptime::Uptime() : dc_measurements::Measurement()
{
}

Uptime::~Uptime() = default;

json Uptime::collect()
{
  auto uptime = system_.upTime();

  json data_json;
  data_json["time"] = uptime;
  return data_json;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Uptime, dc_core::Measurement)
