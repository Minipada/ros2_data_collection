// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/thermal.hpp"

#include <filesystem>
#include <fstream>

namespace dc_measurements
{

namespace
{
bool readZone(const std::string& zone_path, std::string& type_out, double& temp_celsius_out)
{
  std::ifstream type_file(zone_path + "/type");
  std::ifstream temp_file(zone_path + "/temp");
  if (!type_file.is_open() || !temp_file.is_open())
  {
    return false;
  }

  std::getline(type_file, type_out);
  std::string temp_line;
  std::getline(temp_file, temp_line);

  try
  {
    // /sys/class/thermal/thermal_zone*/temp is in millidegrees Celsius.
    temp_celsius_out = std::stol(temp_line) / 1000.0;
  }
  catch (const std::exception&)
  {
    return false;
  }

  return !type_out.empty();
}
}  // namespace

Thermal::Thermal() : dc_measurements::Measurement()
{
}

Thermal::~Thermal() = default;

void Thermal::onConfigure()
{
  auto node = getNode();
  base_path_ = dc_util::get_str_type_param(node, measurement_name_, "base_path", "/sys/class/thermal");
  zones_ = dc_util::get_str_array_type_param(node, measurement_name_, "zones", std::vector<std::string>{});

  // Deliberately don't touch the filesystem (or throw) here: a missing/unreadable
  // /sys/class/thermal (e.g. a container without the host's thermal sysfs mounted, or a
  // platform that doesn't expose one) must not fail activation. collect() re-discovers zones
  // on every poll instead, and simply reports whatever zones it can currently read.
}

void Thermal::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "thermal.json");
  }
}

std::vector<std::string> Thermal::discoverZones()
{
  std::vector<std::string> zones;
  std::error_code ec;
  for (auto it = std::filesystem::directory_iterator(base_path_, ec);
       !ec && it != std::filesystem::directory_iterator(); it.increment(ec))
  {
    std::string name = it->path().filename().string();
    if (name.rfind("thermal_zone", 0) == 0)
    {
      zones.push_back(name);
    }
  }

  return zones;
}

dc_interfaces::msg::StringStamped Thermal::collect()
{
  std::vector<std::string> zones = zones_.empty() ? discoverZones() : zones_;

  json data_json = json::object();
  for (const auto& zone : zones)
  {
    std::string type;
    double temp_celsius;
    if (readZone(base_path_ + "/" + zone, type, temp_celsius))
    {
      data_json[type] = temp_celsius;
    }
    else
    {
      RCLCPP_DEBUG_STREAM(logger_, "Could not read thermal zone '" << zone << "' under '" << base_path_ << "'");
    }
  }

  if (data_json.empty())
  {
    // No zone could be read (e.g. base_path missing/unreadable, or every zone failed):
    // nothing to report this cycle, same "silent, will retry next poll" contract as other
    // hardware-optional plugins (SerialInterface) use for a not-currently-available source.
    return dc_interfaces::msg::StringStamped();
  }

  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  msg.data = data_json.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Thermal, dc_core::Measurement)
