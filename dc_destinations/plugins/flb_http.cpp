#include "dc_destinations/plugins/flb_http.hpp"

namespace dc_destinations
{

FlbHTTP::FlbHTTP() : dc_destinations::FlbDestination()
{
}

void FlbHTTP::onConfigure()
{
  auto node = getNode();

  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".host", rclcpp::ParameterValue("127.0.0.1"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".port", rclcpp::ParameterValue(80));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".uri", rclcpp::ParameterValue("/"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".format", rclcpp::ParameterValue("json"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".header", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".headers_key", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".header_tag", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".json_date_key",
                                               rclcpp::ParameterValue("date"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".json_date_format",
                                               rclcpp::ParameterValue("double"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".http_user", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".http_passwd", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".log_response_payload",
                                               rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".allow_duplicated_headers",
                                               rclcpp::ParameterValue(true));
  node->get_parameter(destination_name_ + ".host", host_);
  node->get_parameter(destination_name_ + ".body_key", body_key_);
  node->get_parameter(destination_name_ + ".port", port_);
  node->get_parameter(destination_name_ + ".uri", uri_);
  node->get_parameter(destination_name_ + ".format", format_);
  node->get_parameter(destination_name_ + ".header", header_);
  node->get_parameter(destination_name_ + ".headers_key", headers_key_);
  node->get_parameter(destination_name_ + ".header_tag", header_tag_);
  node->get_parameter(destination_name_ + ".json_date_key", json_date_key_);
  node->get_parameter(destination_name_ + ".http_user", http_user_);
  node->get_parameter(destination_name_ + ".http_passwd", http_passwd_);
  node->get_parameter(destination_name_ + ".log_response_payload", log_response_payload_);
  node->get_parameter(destination_name_ + ".allow_duplicated_headers", allow_duplicated_headers_);
}

void FlbHTTP::initFlbOutputPlugin()
{
  out_ffd_ = flb_output(ctx_, "http", nullptr);
  flb_output_set(ctx_, out_ffd_, "match", destination_name_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "body_key", body_key_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "host", host_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "port", std::to_string(port_).c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "uri", uri_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "format", format_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "header", header_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "headers_key", headers_key_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "header_tag", header_tag_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "json_date_key", json_date_key_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "http_user", http_user_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "http_passwd", http_passwd_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "log_response_payload", dc_util::boolToString(log_response_payload_), nullptr);
  flb_output_set(ctx_, out_ffd_, "allow_duplicated_headers", dc_util::boolToString(allow_duplicated_headers_), nullptr);
  if (out_ffd_ == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot initialize Fluent Bit output http plugin");
  }
}

FlbHTTP::~FlbHTTP() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbHTTP, dc_core::Destination)
