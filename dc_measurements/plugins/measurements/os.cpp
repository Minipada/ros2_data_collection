#include "dc_measurements/plugins/measurements/os.hpp"

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

unsigned long long OS::getTotalSystemMemory()
{
  long pages = sysconf(_SC_PHYS_PAGES);
  long page_size = sysconf(_SC_PAGE_SIZE);
  return pages * page_size;
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
  data_json["cpus"] = Processor().numCpus();
  auto mem_total = getTotalSystemMemory();
  // Convert to Gb
  double mem_total_gb = (((mem_total / 1024.0) / 1024.0) / 1024.0);
  // Round to 2 decimals
  data_json["memory"] = std::ceil(mem_total_gb * 100.0) / 100.0;

  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::OS, dc_core::Measurement)
