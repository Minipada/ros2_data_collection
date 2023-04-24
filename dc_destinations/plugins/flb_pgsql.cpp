#include "dc_destinations/plugins/flb_pgsql.hpp"

namespace dc_destinations
{

FlbPgSQL::FlbPgSQL() : dc_destinations::FlbDestination()
{
}

void FlbPgSQL::onConfigure()
{
  auto node = getNode();
  char username[MAX_USERID_LENGTH];

  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".host", rclcpp::ParameterValue("127.0.0.1"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".port", rclcpp::ParameterValue(5432));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".user",
                                               rclcpp::ParameterValue(cuserid(username)));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".password", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".database",
                                               rclcpp::ParameterValue(cuserid(username)));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".table", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".timestamp_Key",
                                               rclcpp::ParameterValue("date"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".async", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".min_pool_size", rclcpp::ParameterValue("1"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".max_pool_size", rclcpp::ParameterValue("4"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".cockroachdb", rclcpp::ParameterValue(false));
  node->get_parameter(destination_name_ + ".host", host_);
  node->get_parameter(destination_name_ + ".port", port_);
  node->get_parameter(destination_name_ + ".user", user_);
  node->get_parameter(destination_name_ + ".password", password_);
  node->get_parameter(destination_name_ + ".database", database_);
  node->get_parameter(destination_name_ + ".table", table_);
  node->get_parameter(destination_name_ + ".timestamp_key", timestamp_key_);
  node->get_parameter(destination_name_ + ".async", async_);
  node->get_parameter(destination_name_ + ".min_pool_size", min_pool_size_);
  node->get_parameter(destination_name_ + ".max_pool_size", max_pool_size_);
  node->get_parameter(destination_name_ + ".cockroachdb", cockroachdb_);
}

void FlbPgSQL::initFlbOutputPlugin()
{
  out_ffd_ = flb_output(ctx_, "pgsql", nullptr);
  flb_output_set(ctx_, out_ffd_, "match", destination_name_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "host", host_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "port", std::to_string(port_).c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "user", user_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "password", password_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "database", database_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "table", table_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "timestamp_key", timestamp_key_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "async", dc_util::boolToString(async_), nullptr);
  flb_output_set(ctx_, out_ffd_, "min_pool_size", min_pool_size_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "max_pool_size", max_pool_size_.c_str(), nullptr);
  flb_output_set(ctx_, out_ffd_, "cockroachdb", dc_util::boolToString(cockroachdb_), nullptr);

  if (out_ffd_ == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot initialize Fluent Bit output pgsql plugin");
  }
}

FlbPgSQL::~FlbPgSQL() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbPgSQL, dc_core::Destination)
