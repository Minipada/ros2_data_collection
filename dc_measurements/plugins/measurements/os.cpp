#include "os.hpp"

namespace dc_measurements
{
namespace lp = LinuxParser;

OS::OS() : dc_measurements::Measurement()
{
}

OS::~OS() = default;

void OS::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "os.json");
  }
}

dc_interfaces::msg::StringStamped OS::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  json data_json;
  data_json["os"] = lp::OperatingSystem();
  data_json["kernel"] = lp::Kernel();
  data_json["cpus"] = Processor().NumCpus();
  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::OS, dc_core::Measurement)
