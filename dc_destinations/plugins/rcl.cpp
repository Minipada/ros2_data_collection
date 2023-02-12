#include "dc_destinations/plugins/rcl.hpp"

#include <chrono>
#include <memory>

namespace dc_destinations
{

Rcl::Rcl() : dc_destinations::Destination()
{
}

void Rcl::sendData(dc_interfaces::msg::StringStamped::SharedPtr msg)
{
  json data = json::parse(msg->data);
  if (!custom_params_.empty())
  {
    data.update(custom_params_);
  }
  data.erase("tags");

  RCLCPP_INFO(logger_, "%s", data.dump().c_str());
}

Rcl::~Rcl() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::Rcl, dc_core::Destination)
