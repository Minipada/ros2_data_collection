#include "dc_demos/plugins/measurements/uptime_custom.hpp"

namespace dc_demos
{

void UptimeCustom::onFailedValidation(json data_json)
{
  (void)data_json;
  RCLCPP_INFO(logger_, "Callback! Validation failed for uptime custom");
}

void UptimeCustom::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_demos", "uptime_custom.json");
  }
}

}  // namespace dc_demos

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_demos::UptimeCustom, dc_core::Measurement)
