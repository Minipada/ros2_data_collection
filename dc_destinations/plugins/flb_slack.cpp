#include "dc_destinations/plugins/flb_slack.hpp"

namespace dc_destinations
{

FlbSlack::FlbSlack() : dc_destinations::FlbDestination()
{
}

void FlbSlack::onConfigure()
{
  auto node = getNode();

  nav2_util::declare_parameter_if_not_declared(node, destination_name_ + ".webhook", rclcpp::PARAMETER_STRING);
  node->get_parameter(destination_name_ + ".webhook", webhook_);
}

void FlbSlack::initFlbOutputPlugin()
{
  out_ffd_ = flb_output(ctx_, "slack", nullptr);
  flb_output_set(ctx_, out_ffd_, "webhook", webhook_.c_str(), nullptr);
  if (out_ffd_ == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot initialize Fluent Bit slack stdout plugin");
  }
}

FlbSlack::~FlbSlack() = default;

}  // namespace dc_destinations

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_destinations::FlbSlack, dc_core::Destination)
