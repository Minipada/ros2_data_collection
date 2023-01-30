#include "dc_measurements/plugins/measurements/string_stamped.hpp"

namespace dc_measurements
{

StringStamped::StringStamped() : dc_measurements::Measurement()
{
}

StringStamped::~StringStamped() = default;

// void StringStamped::onConfigure()
// {
//   subscription_ = node->create_subscription<dc_interfaces::msg::StringStamped>(
//       cmd_vel_topic_.c_str(), 10, std::bind(&StringStamped::dataCb, this, std::placeholders::_1));
// }

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
