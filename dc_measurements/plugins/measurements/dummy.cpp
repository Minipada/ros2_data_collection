#include "dc_measurements/plugins/measurements/dummy.hpp"

namespace dc_measurements
{

Dummy::Dummy() : dc_measurements::Measurement()
{
}

Dummy::~Dummy() = default;

void Dummy::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "dummy.json");
  }
}

void Dummy::onConfigure()
{
  auto node = getNode();
  record_ = dc_util::get_str_type_param(node, measurement_name_, "record", "{\"message\":\"Hello from ROS 2 DC\"}");
}

dc_interfaces::msg::StringStamped Dummy::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;

  try
  {
    json data_json = json::parse(record_);
    msg.data = data_json.dump(-1, ' ', true);
  }
  catch (json::parse_error& ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Could not parse record as JSON: " << record_);
  }

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Dummy, dc_core::Measurement)
