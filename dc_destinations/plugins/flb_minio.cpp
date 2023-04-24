#include "dc_destinations/plugins/flb_minio.hpp"

namespace dc_destinations
{

FlbMinIO::FlbMinIO() : dc_destinations::FlbDestination()
{
}

void FlbMinIO::onConfigure()
{
  auto node = getNode();

  std::string package_directory = ament_index_cpp::get_package_prefix("fluent_bit_plugins");
  plugin_path_default_ = package_directory + "/lib/out_minio.so";
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".plugin_path",
                                               rclcpp::ParameterValue(plugin_path_default_));

  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".verbose_plugin",
                                               rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".endpoint",
                                               rclcpp::ParameterValue("127.0.0.1:9000"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".access_key_id", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".secret_access_key", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".use_ssl", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".create_bucket", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".bucket", rclcpp::ParameterValue("dc_bucket"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".upload_fields",
                                               rclcpp::PARAMETER_STRING_ARRAY);
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".src_fields", rclcpp::PARAMETER_STRING_ARRAY);

  node->get_parameter(destination_name_ + ".verbose_plugin", verbose_plugin_);
  node->get_parameter(destination_name_ + ".plugin_path", plugin_path_);
  node->get_parameter(destination_name_ + ".endpoint", endpoint_);
  node->get_parameter(destination_name_ + ".access_key_id", access_key_id_);
  node->get_parameter(destination_name_ + ".secret_access_key", secret_access_key_);
  node->get_parameter(destination_name_ + ".use_ssl", use_ssl_);
  node->get_parameter(destination_name_ + ".create_bucket", create_bucket_);
  node->get_parameter(destination_name_ + ".bucket", bucket_);
  node->get_parameter(destination_name_ + ".upload_fields", upload_fields_);
  node->get_parameter(destination_name_ + ".src_fields", src_fields_);
}

void FlbMinIO::initFlbOutputPlugin()
{
  RCLCPP_INFO(logger_, "Loading minio as GO proxy output %s", plugin_path_.c_str());
  if (flb_plugin_load_router(strdup(plugin_path_.c_str()), ctx_->config) != 0)
  {
    flb_error("[plugin] error loading GO proxy plugin: %s", plugin_path_.c_str());
    throw std::runtime_error("Cannot load plugin");
  }

  out_ffd_ = flb_output(ctx_, "minio", nullptr);
  flb_output_set(ctx_, out_ffd_, "match", destination_name_.c_str(), nullptr);

  flb_output_set(ctx_, out_ffd_, "verbose", dc_util::boolToString(verbose_plugin_), nullptr);
  flb_output_set(ctx_, out_ffd_, "retry_limit", "false", nullptr);
  flb_output_set(ctx_, out_ffd_, "endpoint", endpoint_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "access_key_id", access_key_id_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "secret_access_key", secret_access_key_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "use_ssl", dc_util::boolToString(use_ssl_), nullptr);
  flb_output_set(ctx_, out_ffd_, "create_bucket", dc_util::boolToString(create_bucket_), nullptr);
  flb_output_set(ctx_, out_ffd_, "bucket", bucket_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "src_fields", dc_util::to_space_separated_string(src_fields_).c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "upload_fields", dc_util::to_space_separated_string(upload_fields_).c_str(), nullptr);
}

FlbMinIO::~FlbMinIO() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbMinIO, dc_core::Destination)
