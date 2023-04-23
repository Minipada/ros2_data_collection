#include "dc_measurements/plugins/measurements/string_stamped.hpp"

namespace dc_measurements
{

StringStamped::StringStamped() : dc_measurements::Measurement()
{
}

StringStamped::~StringStamped() = default;

void StringStamped::onConfigure()
{
  auto node = getNode();
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".topic", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".timer_based", rclcpp::ParameterValue(true));
  node->get_parameter(measurement_name_ + ".topic", topic_);
  node->get_parameter(measurement_name_ + ".timer_based", timer_based_);

  subscription_ = node->create_subscription<dc_interfaces::msg::StringStamped>(
      topic_.c_str(), 10, std::bind(&StringStamped::dataCb, this, std::placeholders::_1));
}

void StringStamped::setValidationSchema()
{
}

void StringStamped::dataCb(dc_interfaces::msg::StringStamped::SharedPtr  /*msg*/)
{
  last_data_ = *msg;
  if (!timer_based_)
  {
    publishFromMsg(*msg);
  }
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
