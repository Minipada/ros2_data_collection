#include "dc_measurements/plugins/measurements/string_stamped.hpp"

namespace dc_measurements
{

StringStamped::StringStamped() : dc_measurements::Measurement()
{
}

StringStamped::~StringStamped() = default;

void StringStamped::setValidationSchema()
{
}

void StringStamped::dataCb(dc_interfaces::msg::StringStamped::SharedPtr msg)
{
  last_data_ = *msg;
}

dc_interfaces::msg::StringStamped StringStamped::collect()
{
  dc_interfaces::msg::StringStamped msg = last_data_;
  last_data_ = dc_interfaces::msg::StringStamped();

  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::StringStamped, dc_core::Measurement)
