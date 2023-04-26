#include "dc_destinations/plugins/flb_file.hpp"

namespace dc_destinations
{

FlbFile::FlbFile() : dc_destinations::FlbDestination()
{
}

void FlbFile::onConfigure()
{
  auto node = getNode();

  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".path", rclcpp::ParameterValue("$HOME/data"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".file", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".format", rclcpp::ParameterValue("out_file"));
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".mkdir", rclcpp::ParameterValue(true));

  node->get_parameter(destination_name_ + ".path", path_);
  path_ = dc_util::expand_env(path_);
  node->get_parameter(destination_name_ + ".file", file_);
  node->get_parameter(destination_name_ + ".format", format_);
  node->get_parameter(destination_name_ + ".mkdir", mkdir_);

  std::string default_delimiter = "";
  if (format_ == "ltsv")
  {
    default_delimiter = "\t";
  }
  else if (format_ == "csv")
  {
    default_delimiter = ",";
  }
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".delimiter",
                                               rclcpp::ParameterValue(default_delimiter));
  node->get_parameter(destination_name_ + ".delimiter", delimiter_);
  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".label_delimiter",
                                               rclcpp::ParameterValue("."));
  node->get_parameter(destination_name_ + ".label_delimiter", label_delimiter_);
}

void FlbFile::initFlbOutputPlugin()
{
  /* Enable output plugin 'stdout' (print records to the standard output) */
  out_ffd_ = flb_output(ctx_, "file", NULL);
  flb_output_set(ctx_, out_ffd_, "match", destination_name_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "path", path_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "file", file_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "format", format_.c_str(), NULL);
  flb_output_set(ctx_, out_ffd_, "mkdir", dc_util::boolToString(mkdir_), NULL);
  if (out_ffd_ == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot initialize Fluent Bit output file plugin");
  }
}

FlbFile::~FlbFile() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbFile, dc_core::Destination)
