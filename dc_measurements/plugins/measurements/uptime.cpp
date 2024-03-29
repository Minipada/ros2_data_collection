#include "dc_measurements/plugins/measurements/uptime.hpp"

namespace dc_measurements
{
namespace lp = LinuxParser;

Uptime::Uptime() : dc_measurements::Measurement()
{
}

Uptime::~Uptime() = default;

void Uptime::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "uptime.json");
  }
}

dc_interfaces::msg::StringStamped Uptime::collect()
{
  auto uptime = system_.upTime();

  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  json data_json;
  data_json["time"] = uptime;

  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Uptime, dc_core::Measurement)
