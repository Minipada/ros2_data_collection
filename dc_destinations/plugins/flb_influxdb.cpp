#include "dc_destinations/plugins/flb_influxdb.hpp"

namespace dc_destinations
{

FlbInfluxDB::FlbInfluxDB() : dc_destinations::FlbDestination()
{
}

void FlbInfluxDB::onConfigure()
{
  auto node = getNode();

  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".host", rclcpp::ParameterValue("127.0.0.1"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".port", rclcpp::ParameterValue(8086));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".database",
                                               rclcpp::ParameterValue("fluentbit"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".bucket", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".org", rclcpp::ParameterValue("fluent"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".sequence_tag",
                                               rclcpp::ParameterValue("_seq"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".http_user", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".http_password", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".http_token", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".tag_keys",
                                               rclcpp::ParameterValue(std::vector<std::string>()));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".auto_tags", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".tags_list_enabled",
                                               rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".tags_list_key",
                                               rclcpp::ParameterValue("tags"));
  node->get_parameter(destination_name_ + ".host", host_);
  node->get_parameter(destination_name_ + ".port", port_);
  node->get_parameter(destination_name_ + ".database", database_);
  node->get_parameter(destination_name_ + ".bucket", bucket_);
  node->get_parameter(destination_name_ + ".org", org_);
  node->get_parameter(destination_name_ + ".sequence_tag", sequence_tag_);
  node->get_parameter(destination_name_ + ".http_user", http_user_);
  node->get_parameter(destination_name_ + ".http_password", http_password_);
  node->get_parameter(destination_name_ + ".http_token", http_token_);
  node->get_parameter(destination_name_ + ".tag_keys", tag_keys_);
  node->get_parameter(destination_name_ + ".auto_tags", auto_tags_);
}

void FlbInfluxDB::initFlbOutputPlugin()
{
  out_ffd_ = flb_output(ctx_, "influxdb", NULL);
  flb_output_set(ctx_, out_ffd_, "match", destination_name_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "host", host_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "port", std::to_string(port_), NULL);
  flb_output_set(ctx_, out_ffd_, "database", database_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "bucket", bucket_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "org", org_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "sequence_tag", sequence_tag_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "http_user", http_user_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "http_passwd", http_password_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "http_token", http_token_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "tag_keys", dc_util::join(tag_keys_, " ").c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "auto_tags", dc_util::boolToString(auto_tags_), NULL);
  if (out_ffd_ == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot initialize Fluent Bit output influxdb plugin");
  }
}

FlbInfluxDB::~FlbInfluxDB() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbInfluxDB, dc_core::Destination)
