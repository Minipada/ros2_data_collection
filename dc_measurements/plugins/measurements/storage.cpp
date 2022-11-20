#include "storage.hpp"

#include <chrono>
#include <memory>

namespace dc_measurements
{

Storage::Storage() : dc_measurements::Measurement()
{
}

void Storage::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".path", rclcpp::PARAMETER_STRING);
  node->get_parameter(measurement_name_ + ".path", path_);

  full_path_ = dc_util::expand_env(path_);

  RCLCPP_INFO(logger_, "Storage path expanded to: %s", full_path_.c_str());
  if (full_path_.empty())
  {
    throw std::runtime_error{ "Failed to configure storage node. Need either to set env or path parameter" };
  }
  else {}
}

void Storage::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "storage.json");
  }
}

Storage::~Storage() = default;

dc_interfaces::msg::StringStamped Storage::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  json data_json;
  const std::filesystem::space_info si = std::filesystem::space(full_path_);
  data_json["free_percent"] = (float)si.available / si.capacity * 100;
  data_json["free"] = static_cast<std::intmax_t>(si.free);
  data_json["capacity"] = static_cast<std::intmax_t>(si.capacity);
  msg.data = data_json.dump(-1, ' ', true);

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Storage, dc_core::Measurement)
