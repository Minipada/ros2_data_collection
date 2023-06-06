#include "dc_destinations/plugins/flb_files_metrics.hpp"

namespace dc_destinations
{

FlbFilesMetrics::FlbFilesMetrics() : dc_destinations::FlbDestination()
{
}

void FlbFilesMetrics::onConfigure()
{
  auto node = getNode();

  std::string package_directory = ament_index_cpp::get_package_prefix("fluent_bit_plugins");
  plugin_path_default_ = package_directory + "/lib/flb-out_files_metrics.so";
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".plugin_path",
                                               rclcpp::ParameterValue(plugin_path_default_));
  node->get_parameter(destination_name_ + ".plugin_path", plugin_path_);

  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".file_storage",
                                               rclcpp::PARAMETER_STRING_ARRAY);
  node->get_parameter(destination_name_ + ".file_storage", file_storage_);
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".db_type", rclcpp::ParameterValue("pgsql"));
  node->get_parameter(destination_name_ + ".db_type", db_type_);
  node->get_parameter(destination_name_ + ".delete_when_sent", delete_when_sent_);
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".delete_when_sent",
                                               rclcpp::ParameterValue(true));
  node->get_parameter(destination_name_ + ".delete_when_sent", delete_when_sent_);

  if (std::find(file_storage_.begin(), file_storage_.end(), "minio") != file_storage_.end())
  {
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".minio.endpoint",
                                                 rclcpp::ParameterValue("127.0.0.1:9000"));
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".minio.access_key_id",
                                                 rclcpp::PARAMETER_STRING);
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".minio.secret_access_key",
                                                 rclcpp::PARAMETER_STRING);
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".minio.use_ssl",
                                                 rclcpp::ParameterValue(true));
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".minio.bucket",
                                                 rclcpp::ParameterValue("dc_bucket"));
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".minio.upload_fields",
                                                 rclcpp::PARAMETER_STRING_ARRAY);
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".minio.src_fields",
                                                 rclcpp::PARAMETER_STRING_ARRAY);

    node->get_parameter(destination_name_ + ".minio.endpoint", minio_endpoint_);
    node->get_parameter(destination_name_ + ".minio.access_key_id", minio_access_key_id_);
    node->get_parameter(destination_name_ + ".minio.secret_access_key", minio_secret_access_key_);
    node->get_parameter(destination_name_ + ".minio.use_ssl", minio_use_ssl_);
    node->get_parameter(destination_name_ + ".minio.bucket", minio_bucket_);
    node->get_parameter(destination_name_ + ".minio.upload_fields", minio_upload_fields_);
    node->get_parameter(destination_name_ + ".minio.src_fields", minio_src_fields_);
  }
  if (std::find(file_storage_.begin(), file_storage_.end(), "s3") != file_storage_.end())
  {
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".s3.endpoint", rclcpp::PARAMETER_STRING);
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".s3.access_key_id",
                                                 rclcpp::PARAMETER_STRING);
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".s3.secret_access_key",
                                                 rclcpp::PARAMETER_STRING);
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".s3.bucket", rclcpp::PARAMETER_STRING);
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".s3.upload_fields",
                                                 rclcpp::PARAMETER_STRING_ARRAY);
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".s3.src_fields",
                                                 rclcpp::PARAMETER_STRING_ARRAY);

    node->get_parameter(destination_name_ + ".s3.endpoint", s3_endpoint_);
    node->get_parameter(destination_name_ + ".s3.access_key_id", s3_access_key_id_);
    node->get_parameter(destination_name_ + ".s3.secret_access_key", s3_secret_access_key_);
    node->get_parameter(destination_name_ + ".s3.bucket", s3_bucket_);
    node->get_parameter(destination_name_ + ".s3.upload_fields", s3_upload_fields_);
    node->get_parameter(destination_name_ + ".s3.src_fields", s3_src_fields_);
  }

  if (db_type_ == "pgsql")
  {
    char username[MAX_USERID_LENGTH];

    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".pgsql.host",
                                                 rclcpp::ParameterValue("127.0.0.1"));
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".pgsql.port",
                                                 rclcpp::ParameterValue("5432"));
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".pgsql.user",
                                                 rclcpp::ParameterValue(cuserid(username)));
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".pgsql.password",
                                                 rclcpp::ParameterValue(""));
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".pgsql.database",
                                                 rclcpp::ParameterValue(cuserid(username)));
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".pgsql.table",
                                                 rclcpp::ParameterValue("pg_table"));
    nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".pgsql.ssl", rclcpp::ParameterValue(false));
    node->get_parameter(destination_name_ + ".pgsql.host", pgsql_host_);
    node->get_parameter(destination_name_ + ".pgsql.port", pgsql_port_);
    node->get_parameter(destination_name_ + ".pgsql.user", pgsql_user_);
    node->get_parameter(destination_name_ + ".pgsql.password", pgsql_password_);
    node->get_parameter(destination_name_ + ".pgsql.database", pgsql_database_);
    node->get_parameter(destination_name_ + ".pgsql.table", pgsql_table_);
    node->get_parameter(destination_name_ + ".pgsql.ssl", pgsql_use_ssl_);
  }
}

void FlbFilesMetrics::initFlbOutputPlugin()
{
  RCLCPP_INFO(logger_, "Loading flb_metrics as GO proxy output %s", plugin_path_.c_str());
  if (flb_plugin_load_router(strdup(plugin_path_.c_str()), ctx_->config) != 0)
  {
    flb_error("[plugin] error loading GO proxy plugin: %s", plugin_path_.c_str());
    throw std::runtime_error("Cannot load plugin");
  }

  out_ffd_ = flb_output(ctx_, "files_metrics", nullptr);
  flb_output_set(ctx_, out_ffd_, "match", destination_name_.c_str(), nullptr);

  flb_output_set(ctx_, out_ffd_, "retry_limit", "false", nullptr);
  flb_output_set(ctx_, out_ffd_, "file_storage", dc_util::to_space_separated_string(file_storage_).c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "db_type", db_type_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "delete_when_sent", dc_util::boolToString(delete_when_sent_), nullptr);

  if (std::find(file_storage_.begin(), file_storage_.end(), "minio") != file_storage_.end())
  {
    flb_output_set(ctx_, out_ffd_, "minio_endpoint", minio_endpoint_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "minio_access_key_id", minio_access_key_id_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "minio_secret_access_key", minio_secret_access_key_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "minio_use_ssl", dc_util::boolToString(minio_use_ssl_), nullptr);
    flb_output_set(ctx_, out_ffd_, "minio_create_bucket", minio_create_bucket_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "minio_bucket", minio_bucket_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "minio_src_fields", dc_util::to_space_separated_string(minio_src_fields_).c_str(),
                   nullptr);
    flb_output_set(ctx_, out_ffd_, "minio_upload_fields",
                   dc_util::to_space_separated_string(minio_upload_fields_).c_str(), nullptr);
  }

  if (std::find(file_storage_.begin(), file_storage_.end(), "s3") != file_storage_.end())
  {
    flb_output_set(ctx_, out_ffd_, "s3_endpoint", s3_endpoint_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "s3_access_key_id", s3_access_key_id_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "s3_secret_access_key", s3_secret_access_key_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "s3_create_bucket", s3_create_bucket_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "s3_bucket", s3_bucket_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "s3_src_fields", dc_util::to_space_separated_string(s3_src_fields_).c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "s3_upload_fields", dc_util::to_space_separated_string(s3_upload_fields_).c_str(),
                   nullptr);
  }

  if (db_type_ == "pgsql")
  {
    std::string pgsql_use_ssl_str = pgsql_use_ssl_ ? "require" : "disable";
    flb_output_set(ctx_, out_ffd_, "pgsql_host", pgsql_host_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "pgsql_port", pgsql_port_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "pgsql_user", pgsql_user_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "pgsql_password", pgsql_password_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "pgsql_database", pgsql_database_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "pgsql_table", pgsql_table_.c_str(), nullptr);
    flb_output_set(ctx_, out_ffd_, "pgsql_use_ssl", pgsql_use_ssl_str.c_str(), nullptr);
  }
}

FlbFilesMetrics::~FlbFilesMetrics() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbFilesMetrics, dc_core::Destination)
