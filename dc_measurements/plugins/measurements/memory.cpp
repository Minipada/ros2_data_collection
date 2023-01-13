#include "memory.hpp"

namespace dc_measurements
{

Memory::Memory() : dc_measurements::Measurement()
{
}

Memory::~Memory() = default;

void Memory::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "memory.json");
  }
}

dc_interfaces::msg::StringStamped Memory::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  json data_json;
  data_json["used"] = System().MemoryUtilization() * 100;
  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Memory, dc_core::Measurement)
