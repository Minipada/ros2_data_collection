#include "dc_measurements/measurement_server.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace measurement_server
{

MeasurementServer::MeasurementServer(const rclcpp::NodeOptions& options)
  : nav2_util::LifecycleNode("measurement_server", "", options), plugin_loader_("dc_core", "dc_core::Measurement")
{
  declare_parameter("measurement_plugins", measurement_plugins_);
  get_parameter("measurement_plugins", measurement_ids_);
}

MeasurementServer::~MeasurementServer()
{
  measurements_.clear();
}

nav2_util::CallbackReturn MeasurementServer::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  auto node = shared_from_this();
  timer_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  measurement_types_.resize(measurement_ids_.size());
  measurement_topic_outputs_.resize(measurement_ids_.size());
  measurement_polling_interval_.resize(measurement_ids_.size());
  measurement_debug_.resize(measurement_ids_.size());
  measurement_enable_validator_.resize(measurement_ids_.size());
  measurement_json_schema_path_.resize(measurement_ids_.size());
  measurement_group_key_.resize(measurement_ids_.size());

  if (!loadMeasurementPlugins())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

bool MeasurementServer::loadMeasurementPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != measurement_ids_.size(); i++)
  {
    // Mandatory parameters
    measurement_types_[i] = dc_util::get_str_type_param(node, measurement_ids_[i], "plugin");
    measurement_group_key_[i] = dc_util::get_str_type_param(node, measurement_ids_[i], "group_key");

    // Optional parameters
    measurement_topic_outputs_[i] = dc_util::get_str_type_param(node, measurement_ids_[i], "topic_output",
                                                                std::string("/dc/measurement/") + measurement_ids_[i]);
    measurement_polling_interval_[i] = dc_util::get_int_type_param(node, measurement_ids_[i], "polling_interval", 1000);
    measurement_debug_[i] = dc_util::get_bool_type_param(node, measurement_ids_[i], "debug", false);
    measurement_enable_validator_[i] =
        dc_util::get_bool_type_param(node, measurement_ids_[i], "enable_validator", true);
    measurement_json_schema_path_[i] = dc_util::get_str_type_param(node, measurement_ids_[i], "json_schema_path", "");

    try
    {
      RCLCPP_INFO(get_logger(),
                  "Creating measurement plugin %s: Type %s, Group key: %s, Polling interval: %d, Debug: %d, Validator "
                  "enabled: %d, "
                  "Schema path: %s",
                  measurement_ids_[i].c_str(), measurement_types_[i].c_str(), measurement_group_key_[i].c_str(),
                  measurement_polling_interval_[i], (int)measurement_debug_[i], (int)measurement_enable_validator_[i],
                  measurement_json_schema_path_[i].c_str());
      measurements_.push_back(plugin_loader_.createUniqueInstance(measurement_types_[i]));
      measurements_.back()->configure(node, measurement_ids_[i], tf_, measurement_types_[i], measurement_group_key_[i],
                                      measurement_topic_outputs_[i], measurement_polling_interval_[i],
                                      measurement_debug_[i], measurement_enable_validator_[i],
                                      measurement_json_schema_path_[i], timer_cb_group_);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(get_logger(),
                   "Failed to create measurement %s of type %s."
                   " Exception: %s",
                   measurement_ids_[i].c_str(), measurement_types_[i].c_str(), ex.what());
      return false;
    }
  }

  return true;
}

nav2_util::CallbackReturn MeasurementServer::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Activating");
  std::vector<pluginlib::UniquePtr<dc_core::Measurement>>::iterator iter;
  for (iter = measurements_.begin(); iter != measurements_.end(); ++iter)
  {
    (*iter)->activate();
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MeasurementServer::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  std::vector<pluginlib::UniquePtr<dc_core::Measurement>>::iterator iter;
  for (iter = measurements_.begin(); iter != measurements_.end(); ++iter)
  {
    (*iter)->deactivate();
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MeasurementServer::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  tf_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MeasurementServer::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // end namespace measurement_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(measurement_server::MeasurementServer)
