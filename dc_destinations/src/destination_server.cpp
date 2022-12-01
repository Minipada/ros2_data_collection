#include "dc_destinations/destination_server.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace destination_server
{

DestinationServer::DestinationServer(const rclcpp::NodeOptions& options)
  : nav2_util::LifecycleNode("destination_server", "", options), plugin_loader_("dc_core", "dc_core::Destination")
{
  declare_parameter("destination_plugins", default_ids_);
  get_parameter("destination_plugins", destination_ids_);
}

DestinationServer::~DestinationServer()
{
  destinations_.clear();
}

nav2_util::CallbackReturn DestinationServer::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  destination_types_.resize(destination_ids_.size());
  destination_inputs_.resize(destination_ids_.size());
  destination_debug_.resize(destination_ids_.size());

  if (!loadDestinationPlugins())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

bool DestinationServer::loadDestinationPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != destination_ids_.size(); i++)
  {
    // Mandatory parameters
    destination_types_[i] = dc_util::get_str_type_param(node, destination_ids_[i], "plugin");
    destination_inputs_[i] = dc_util::get_str_array_type_param(node, destination_ids_[i], "inputs");
    destination_debug_[i] = dc_util::get_bool_type_param(node, destination_ids_[i], "debug", false);

    try
    {
      RCLCPP_INFO(get_logger(), "Creating destination plugin %s: Type %s", destination_ids_[i].c_str(),
                  destination_types_[i].c_str());
      destinations_.push_back(plugin_loader_.createUniqueInstance(destination_types_[i]));
      destinations_.back()->configure(node, destination_ids_[i], destination_inputs_[i], destination_debug_[i]);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(get_logger(),
                   "Failed to create destination %s of type %s, debug %d"
                   " Exception: %s",
                   destination_ids_[i].c_str(), destination_types_[i].c_str(), (int)destination_debug_[i], ex.what());
      return false;
    }
  }

  return true;
}

nav2_util::CallbackReturn DestinationServer::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Activating");
  std::vector<pluginlib::UniquePtr<dc_core::Destination>>::iterator iter;
  for (iter = destinations_.begin(); iter != destinations_.end(); ++iter)
  {
    (*iter)->activate();
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DestinationServer::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  std::vector<pluginlib::UniquePtr<dc_core::Destination>>::iterator iter;
  for (iter = destinations_.begin(); iter != destinations_.end(); ++iter)
  {
    (*iter)->deactivate();
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DestinationServer::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DestinationServer::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // end namespace destination_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(destination_server::DestinationServer)
