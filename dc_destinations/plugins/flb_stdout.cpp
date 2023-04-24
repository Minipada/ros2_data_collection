#include "dc_destinations/plugins/flb_stdout.hpp"

namespace dc_destinations
{

FlbStdout::FlbStdout() : dc_destinations::FlbDestination()
{
}

void FlbStdout::onConfigure()
{
  auto node = getNode();

  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".format", rclcpp::ParameterValue("json"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".json_date_key",
                                               rclcpp::ParameterValue("date"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".json_date_format",
                                               rclcpp::ParameterValue("double"));
  node->get_parameter(destination_name_ + ".format", format_);
  node->get_parameter(destination_name_ + ".json_date_key", json_date_key_);
  node->get_parameter(destination_name_ + ".json_date_format", json_date_format_);
}

void FlbStdout::initFlbOutputPlugin()
{
  out_ffd_ = flb_output(ctx_, "stdout", nullptr);
  flb_output_set(ctx_, out_ffd_, "match", destination_name_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "format", format_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "json_date_key", json_date_key_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "json_date_format", json_date_format_.c_str(), nullptr);
  if (out_ffd_ == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot initialize Fluent Bit output stdout plugin");
  }
}

FlbStdout::~FlbStdout() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbStdout, dc_core::Destination)
