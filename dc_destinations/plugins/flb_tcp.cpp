#include "dc_destinations/plugins/flb_tcp.hpp"

namespace dc_destinations
{

FlbTCP::FlbTCP() : dc_destinations::FlbDestination()
{
}

void FlbTCP::onConfigure()
{
  auto node = getNode();

  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".host", rclcpp::ParameterValue("127.0.0.1"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".port", rclcpp::ParameterValue(5432));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".format", rclcpp::ParameterValue("json"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".workers", rclcpp::ParameterValue(2));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".tls.active", rclcpp::ParameterValue("off"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".tls.verify", rclcpp::ParameterValue("on"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".tls.debug", rclcpp::ParameterValue(1));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".tls.ca_file", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".tls.crt_file", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".tls.key_file", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".tls.key_passwd", rclcpp::PARAMETER_STRING);
  node->get_parameter(destination_name_ + ".host", host_);
  node->get_parameter(destination_name_ + ".port", port_);
  node->get_parameter(destination_name_ + ".format", format_);
  node->get_parameter(destination_name_ + ".workers", workers_);
  node->get_parameter(destination_name_ + ".tls.active", tls_active_);
  node->get_parameter(destination_name_ + ".tls.verify", tls_verify_);
  node->get_parameter(destination_name_ + ".tls.debug", tls_debug_);
  node->get_parameter(destination_name_ + ".tls.ca_file", tls_ca_file_);
  node->get_parameter(destination_name_ + ".tls.crt_file", tls_crt_file_);
  node->get_parameter(destination_name_ + ".tls.key_file", tls_key_file_);
  node->get_parameter(destination_name_ + ".tls.key_passwd", tls_key_passwd_);
}

void FlbTCP::initFlbOutputPlugin()
{
  out_ffd_ = flb_output(ctx_, "tcp", NULL);
  flb_output_set(ctx_, out_ffd_, "host", host_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "port", std::to_string(port_), NULL);
  flb_output_set(ctx_, out_ffd_, "format", format_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "workers", std::to_string(workers_), NULL);
  flb_output_set(ctx_, out_ffd_, "tls.active", dc_util::boolToString(tls_active_), NULL);
  flb_output_set(ctx_, out_ffd_, "tls.verify", dc_util::boolToString(tls_verify_), NULL);
  flb_output_set(ctx_, out_ffd_, "tls.debug", std::to_string(tls_debug_), NULL);
  flb_output_set(ctx_, out_ffd_, "tls.ca_file", tls_ca_file_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "tls.crt_file", tls_crt_file_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "tls.key_file", tls_key_file_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "tls.key_passwd", tls_key_passwd_.c_str(), NULL);
  if (out_ffd_ == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot initialize Fluent Bit tcp stdout plugin");
  }
}

FlbTCP::~FlbTCP() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbTCP, dc_core::Destination)
