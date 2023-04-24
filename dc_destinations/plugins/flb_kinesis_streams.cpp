#include "dc_destinations/plugins/flb_kinesis_streams.hpp"

namespace dc_destinations
{

FlbKinesisStreams::FlbKinesisStreams() : dc_destinations::FlbDestination()
{
}

void FlbKinesisStreams::onConfigure()
{
  auto node = getNode();

  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".region", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".stream", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".role_arn", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".time_key", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".time_key_format",
                                               rclcpp::ParameterValue("%Y-%m-%dT%H:%M:%S"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".log_key", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".endpoint", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".auto_retry_requests",
                                               rclcpp::ParameterValue("true"));

  node->get_parameter(destination_name_ + ".region", region_);
  node->get_parameter(destination_name_ + ".stream", stream_);
  node->get_parameter(destination_name_ + ".role_arn", role_arn_);
  node->get_parameter(destination_name_ + ".time_key", time_key_);
  node->get_parameter(destination_name_ + ".time_key_format", time_key_format_);
  node->get_parameter(destination_name_ + ".log_key", log_key_);
  node->get_parameter(destination_name_ + ".endpoint", endpoint_);
  node->get_parameter(destination_name_ + ".auto_retry_requests", auto_retry_requests_);
}

void FlbKinesisStreams::initFlbOutputPlugin()
{
  /* Enable output plugin 'stdout' (print records to the standard output) */
  out_ffd_ = flb_output(ctx_, "kinesis_streams", NULL);
  flb_output_set(ctx_, out_ffd_, "match", destination_name_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "stream", stream_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "role_arn", role_arn_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "time_key", time_key_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "time_key_format", time_key_format_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "log_key", log_key_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "endpoint", endpoint_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "auto_retry_requests", dc_util::boolToString(auto_retry_requests_), NULL);

  if (out_ffd_ == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot initialize Fluent Bit output kinesis_streams plugin");
  }
}

FlbKinesisStreams::~FlbKinesisStreams() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbKinesisStreams, dc_core::Destination)
