#include "dc_measurements/plugins/measurements/cpu.hpp"

namespace dc_measurements
{
namespace lp = LinuxParser;

Cpu::Cpu() : dc_measurements::Measurement()
{
}

Cpu::~Cpu() = default;

void Cpu::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".max_processes", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".cpu_min", rclcpp::ParameterValue(5.0));
  node->get_parameter(measurement_name_ + ".max_processes", max_processes_);
  node->get_parameter(measurement_name_ + ".cpu_min", cpu_min_);
}

void Cpu::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "cpu.json");
  }
}

void Cpu::setAverageCpu(json& data_json)
{
  std::vector<float> values;
  int cpu_id{ 0 };
  for (float cpu : system_.cpu().utilization())
  {
    std::string value = std::to_string(cpu * 100).substr(0, 4);
    if (cpu < 0.1 || cpu == 1.0)
    {
      value = std::to_string(cpu * 100).substr(0, 3);
    }
    values.push_back(std::stof(value));
    cpu_id++;
  }
  double sum = std::accumulate(values.begin(), values.end(), 0.0);
  data_json["average"] = sum / values.size();
}

void Cpu::setProcessesUsage(json& data_json)
{
  if (max_processes_ == -1)
  {
    return;
  }
  data_json["sorted"] = json::array();
  int count = 0;
  auto processes = system_.processes();
  for (auto& process : processes)
  {
    if (count == max_processes_)
    {
      break;
    }
    if (cpu_min_ == -1 || process.cpuUtilization() >= cpu_min_)
    {
      json proc_json;
      proc_json["pid"] = process.pid();
      proc_json["user"] = process.user();
      proc_json["cmd"] = process.command();
      proc_json["cpu"] = process.cpuUtilization();
      proc_json["ram"] = process.ram();
      proc_json["uptime"] = process.upTime();
      data_json["sorted"].push_back(proc_json);
      count++;
    }
  }
}

void setProcessesCount(json& data_json)
{
  data_json["processes"] = lp::TotalProcesses();
}

dc_interfaces::msg::StringStamped Cpu::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;

  json data_json;
  setAverageCpu(data_json);
  setProcessesCount(data_json);
  setProcessesUsage(data_json);

  msg.data = data_json.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Cpu, dc_core::Measurement)
