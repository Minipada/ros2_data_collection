#include "dc_destinations/plugins/flb_s3.hpp"

namespace dc_destinations
{

FlbS3::FlbS3() : dc_destinations::FlbDestination()
{
}

void FlbS3::onConfigure()
{
  auto node = getNode();

  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".region", rclcpp::ParameterValue("us-east-1"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".bucket", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".json_date_key",
                                               rclcpp::ParameterValue("date"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".json_date_format",
                                               rclcpp::ParameterValue("iso8601"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".total_file_size",
                                               rclcpp::ParameterValue("100M"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".upload_chunk_size",
                                               rclcpp::ParameterValue("50M"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".upload_timeout",
                                               rclcpp::ParameterValue("10m"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".store_dir",
                                               rclcpp::ParameterValue("/tmp/fluent-bit/s3"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".store_dir_limit_size",
                                               rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".s3_key_format",
                                               rclcpp::ParameterValue("/fluent-bit-logs/$TAG/%Y/%m/%d/%H/%M/%S"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".static_file_path",
                                               rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".s3_key_format_tag_delimiters",
                                               rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".use_put_object",
                                               rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".role_arn", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".endpoint", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".sts_endpoint", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".canned_acl", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".compression", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".content_type", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".send_content_md5",
                                               rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".auto_retry_requests",
                                               rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".log_key", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".preserve_data_ordering",
                                               rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".storage_class", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".retry_limit", rclcpp::ParameterValue(1));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".external_id", rclcpp::ParameterValue(""));
  node->get_parameter(destination_name_ + ".region", region_);
  node->get_parameter(destination_name_ + ".bucket", bucket_);
  node->get_parameter(destination_name_ + ".json_date_key", json_date_key_);
  node->get_parameter(destination_name_ + ".json_date_format", json_date_format_);
  node->get_parameter(destination_name_ + ".total_file_size", total_file_size_);
  node->get_parameter(destination_name_ + ".upload_chunk_size", upload_chunk_size_);
  node->get_parameter(destination_name_ + ".upload_timeout", upload_timeout_);
  node->get_parameter(destination_name_ + ".store_dir", store_dir_);
  node->get_parameter(destination_name_ + ".store_dir_limit_size", store_dir_limit_size_);
  node->get_parameter(destination_name_ + ".s3_key_format", s3_key_format_);
  node->get_parameter(destination_name_ + ".s3_key_format_tag_delimiters", s3_key_format_tag_delimiters_);
  node->get_parameter(destination_name_ + ".static_file_path", static_file_path_);
  node->get_parameter(destination_name_ + ".use_put_object", use_put_object_);
  node->get_parameter(destination_name_ + ".role_arn", role_arn_);
  node->get_parameter(destination_name_ + ".endpoint", endpoint_);
  node->get_parameter(destination_name_ + ".sts_endpoint", sts_endpoint_);
  node->get_parameter(destination_name_ + ".canned_acl", canned_acl_);
  node->get_parameter(destination_name_ + ".compression", compression_);
  node->get_parameter(destination_name_ + ".content_type", content_type_);
  node->get_parameter(destination_name_ + ".send_content_md5", send_content_md5_);
  node->get_parameter(destination_name_ + ".auto_retry_requests", auto_retry_requests_);
  node->get_parameter(destination_name_ + ".log_key", log_key_);
  node->get_parameter(destination_name_ + ".preserve_data_ordering", preserve_data_ordering_);
  node->get_parameter(destination_name_ + ".storage_class", storage_class_);
  node->get_parameter(destination_name_ + ".retry_limit", retry_limit_);
  node->get_parameter(destination_name_ + ".role_arn", role_arn_);
  node->get_parameter(destination_name_ + ".external_id", external_id_);
}

void FlbS3::initFlbOutputPlugin()
{
  /* Enable output plugin 'stdout' (print records to the standard output) */
  out_ffd_ = flb_output(ctx_, "s3", nullptr);
  flb_output_set(ctx_, out_ffd_, "match", destination_name_.c_str(), nullptr);

  flb_output_set(ctx_, out_ffd_, "region", region_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "bucket", bucket_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "json_date_key", json_date_key_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "json_date_format", json_date_format_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "total_file_size", total_file_size_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "upload_chunk_size", upload_chunk_size_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "upload_timeout", upload_timeout_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "store_dir", store_dir_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "store_dir_limit_size", std::to_string(store_dir_limit_size_).c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "s3_key_format", s3_key_format_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "s3_key_format_tag_delimiters", s3_key_format_tag_delimiters_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "static_file_path", dc_util::boolToString(static_file_path_), nullptr);
  flb_output_set(ctx_, out_ffd_, "use_put_object", dc_util::boolToString(use_put_object_), nullptr);
  flb_output_set(ctx_, out_ffd_, "role_arn", role_arn_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "endpoint", endpoint_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "sts_endpoint", sts_endpoint_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "canned_acl", canned_acl_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "compression", compression_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "content_type", content_type_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "send_content_md5", dc_util::boolToString(send_content_md5_), nullptr);
  flb_output_set(ctx_, out_ffd_, "auto_retry_requests", dc_util::boolToString(auto_retry_requests_), nullptr);
  flb_output_set(ctx_, out_ffd_, "log_key", log_key_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "preserve_data_ordering", dc_util::boolToString(preserve_data_ordering_), nullptr);
  flb_output_set(ctx_, out_ffd_, "storage_class", storage_class_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "retry_limit", retry_limit_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "external_id", external_id_.c_str(), nullptr);

  if (out_ffd_ == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot initialize Fluent Bit output s3 plugin");
  }
}

FlbS3::~FlbS3() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbS3, dc_core::Destination)
