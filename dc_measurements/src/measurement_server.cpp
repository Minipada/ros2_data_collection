#include "dc_measurements/measurement_server.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace measurement_server
{

MeasurementServer::MeasurementServer(const rclcpp::NodeOptions& options,
                                     const std::vector<std::string>& measurement_plugins)
  : nav2_util::LifecycleNode("measurement_server", "", options)
  , measurement_plugin_loader_("dc_core", "dc_core::Measurement")
  , condition_plugin_loader_("dc_core", "dc_core::Condition")
{
  declare_parameter("measurement_plugins", measurement_plugins);
  get_parameter("measurement_plugins", measurement_ids_);
  declare_parameter("condition_plugins", std::vector<std::string>());
  get_parameter("condition_plugins", condition_ids_);

  setCustomParameters();

  // Base file saving path
  setBaseSavePath();
}

void MeasurementServer::setBaseSavePath()
{
  declare_parameter("save_local_base_path", "$HOME/ros2/data/%Y/%M/%D/%H");
  get_parameter("save_local_base_path", save_local_base_path_);
  declare_parameter("all_base_path", "");
  get_parameter("all_base_path", all_base_path_);

  save_local_base_path_expanded_ = dc_util::expand_env(save_local_base_path_);
  save_local_base_path_expanded_ = dc_util::expand_values(save_local_base_path_expanded_, this);
  RCLCPP_INFO(get_logger(), "Base save path expanded to %s", save_local_base_path_expanded_.c_str());
  all_base_path_expanded_ = dc_util::expand_env(all_base_path_);
  all_base_path_expanded_ = dc_util::expand_values(all_base_path_expanded_, this);
  RCLCPP_INFO(get_logger(), "All Base path expanded to %s", all_base_path_expanded_.c_str());
}

void MeasurementServer::setCustomParameters()
{
  declare_parameter("custom_str_params", std::vector<std::string>());
  get_parameter("custom_str_params", measurement_custom_str_params_);
  for (auto param = std::begin(measurement_custom_str_params_); param != std::end(measurement_custom_str_params_);
       ++param)
  {
    declare_parameter(*param, "");
    get_parameter(*param, custom_str_params_map_[*param]);
  }
}

MeasurementServer::~MeasurementServer()
{
  measurements_.clear();
}

nav2_util::CallbackReturn MeasurementServer::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  auto node = shared_from_this();

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
  measurement_tags_.resize(measurement_ids_.size());
  measurement_init_collect_.resize(measurement_ids_.size());
  measurement_init_max_measurements_.resize(measurement_ids_.size());
  measurement_include_measurement_name_.resize(measurement_ids_.size());
  measurement_include_measurement_plugin_.resize(measurement_ids_.size());
  measurement_remote_keys_.resize(measurement_ids_.size());
  measurement_remote_prefixes_.resize(measurement_ids_.size());

  measurement_if_all_conditions_.resize(measurement_ids_.size());
  measurement_if_any_conditions_.resize(measurement_ids_.size());
  measurement_if_none_conditions_.resize(measurement_ids_.size());
  measurement_condition_max_measurements_.resize(measurement_ids_.size());

  condition_types_.resize(condition_ids_.size());

  if (!loadConditionPlugins())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (!loadMeasurementPlugins())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

bool MeasurementServer::loadConditionPlugins()
{
  auto node = shared_from_this();
  for (size_t i = 0; i != condition_ids_.size(); i++)
  {
    // Mandatory parameters
    condition_types_[i] = dc_util::get_str_type_param(node, condition_ids_[i], "plugin");
    try
    {
      RCLCPP_INFO(get_logger(), "Creating condition plugin %s: Type %s", condition_ids_[i].c_str(),
                  condition_types_[i].c_str());
      conditions_[condition_ids_[i]] = condition_plugin_loader_.createUniqueInstance(condition_types_[i]);
      conditions_[condition_ids_[i]]->configure(node, condition_ids_[i]);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(get_logger(),
                   "Failed to create condition %s of type %s."
                   " Exception: %s",
                   condition_ids_[i].c_str(), condition_types_[i].c_str(), ex.what());
      return false;
    }
  }

  return true;
}

bool MeasurementServer::loadMeasurementPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != measurement_ids_.size(); i++)
  {
    // Mandatory parameters
    measurement_types_[i] = dc_util::get_str_type_param(node, measurement_ids_[i], "plugin");

    // Optional parameters
    measurement_group_key_[i] = dc_util::get_str_type_param(node, measurement_ids_[i], "group_key", "");
    measurement_topic_outputs_[i] = dc_util::get_str_type_param(node, measurement_ids_[i], "topic_output",
                                                                std::string("/dc/measurement/") + measurement_ids_[i]);
    measurement_polling_interval_[i] = dc_util::get_int_type_param(node, measurement_ids_[i], "polling_interval", 1000);
    measurement_debug_[i] = dc_util::get_bool_type_param(node, measurement_ids_[i], "debug", false);
    measurement_enable_validator_[i] =
        dc_util::get_bool_type_param(node, measurement_ids_[i], "enable_validator", true);
    measurement_json_schema_path_[i] = dc_util::get_str_type_param(node, measurement_ids_[i], "json_schema_path", "");
    measurement_tags_[i] =
        dc_util::get_str_array_type_param(node, measurement_ids_[i], "tags", std::vector<std::string>());
    measurement_init_collect_[i] = dc_util::get_bool_type_param(node, measurement_ids_[i], "init_collect", true);
    measurement_init_max_measurements_[i] =
        dc_util::get_int_type_param(node, measurement_ids_[i], "init_max_measurements", 0);
    measurement_include_measurement_name_[i] =
        dc_util::get_bool_type_param(node, measurement_ids_[i], "include_measurement_name", true);
    measurement_include_measurement_plugin_[i] =
        dc_util::get_bool_type_param(node, measurement_ids_[i], "include_measurement_plugin", false);
    measurement_remote_keys_[i] =
        dc_util::get_str_array_type_param(node, measurement_ids_[i], "remote_keys", std::vector<std::string>());
    measurement_remote_prefixes_[i] =
        dc_util::get_str_array_type_param(node, measurement_ids_[i], "remote_prefixes", std::vector<std::string>());

    measurement_if_all_conditions_[i] =
        dc_util::get_str_array_type_param(node, measurement_ids_[i], "if_all_conditions", std::vector<std::string>());
    measurement_if_any_conditions_[i] =
        dc_util::get_str_array_type_param(node, measurement_ids_[i], "if_any_conditions", std::vector<std::string>());
    measurement_if_none_conditions_[i] =
        dc_util::get_str_array_type_param(node, measurement_ids_[i], "if_none_conditions", std::vector<std::string>());
    measurement_condition_max_measurements_[i] =
        dc_util::get_int_type_param(node, measurement_ids_[i], "condition_max_measurements", 0);

    try
    {
      RCLCPP_INFO_STREAM(get_logger(),
                         "Creating measurement plugin "
                             << measurement_ids_[i].c_str() << ": Type " << measurement_types_[i].c_str()
                             << ", Group key: " << measurement_group_key_[i] << ", Polling interval: "
                             << measurement_polling_interval_[i] << ", Debug: " << (int)measurement_debug_[i]
                             << ", Validator enabled: " << (int)measurement_enable_validator_[i]
                             << ", Schema path: " << measurement_json_schema_path_[i].c_str() << ", Tags: ["
                             << dc_util::join(measurement_tags_[i], ",")
                             << "], Init collect: " << (int)measurement_init_collect_[i]
                             << ", Init Max measurement: " << measurement_init_max_measurements_[i]
                             << ", Include measurement name: " << measurement_include_measurement_name_[i]
                             << ", Include measurement plugin name: " << measurement_include_measurement_plugin_[i]
                             << ", Remote keys: " << dc_util::join(measurement_remote_keys_[i])
                             << ", Remote prefixes: " << dc_util::join(measurement_remote_prefixes_[i])
                             << ", Include measurement plugin name: " << measurement_include_measurement_plugin_[i]
                             << ", Max measurement on condition: " << measurement_condition_max_measurements_[i]
                             << ", If all condition: " << dc_util::join(measurement_if_all_conditions_[i], ",")
                             << ", If any condition: " << dc_util::join(measurement_if_any_conditions_[i], ",")
                             << ", If none condition: " << dc_util::join(measurement_if_none_conditions_[i], ","));

      measurements_.push_back(measurement_plugin_loader_.createUniqueInstance(measurement_types_[i]));
      measurements_.back()->configure(
          node, measurement_ids_[i], conditions_, tf_, measurement_types_[i], measurement_group_key_[i],
          measurement_topic_outputs_[i], measurement_polling_interval_[i], measurement_debug_[i],
          measurement_enable_validator_[i], measurement_json_schema_path_[i], measurement_tags_[i],
          measurement_init_collect_[i], measurement_init_max_measurements_[i], measurement_include_measurement_name_[i],
          measurement_include_measurement_plugin_[i], measurement_condition_max_measurements_[i],
          measurement_if_all_conditions_[i], measurement_if_any_conditions_[i], measurement_if_none_conditions_[i],
          measurement_remote_keys_[i], measurement_remote_prefixes_[i], save_local_base_path_, all_base_path_,
          all_base_path_expanded_, save_local_base_path_expanded_);
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

nav2_util::CallbackReturn MeasurementServer::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
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

nav2_util::CallbackReturn MeasurementServer::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

std::vector<std::string> MeasurementServer::getMeasurementPlugins()
{
  return measurement_ids_;
}

std::vector<std::string> MeasurementServer::getMeasurementTypes()
{
  return measurement_types_;
}

std::vector<std::string> MeasurementServer::getMeasurementGroupKeys()
{
  return measurement_group_key_;
}

std::vector<std::string> MeasurementServer::getMeasurementTopicOutput()
{
  return measurement_topic_outputs_;
}

std::vector<int> MeasurementServer::getMeasurementPollingInterval()
{
  return measurement_polling_interval_;
}

std::vector<bool> MeasurementServer::getMeasurementInitCollect()
{
  return measurement_init_collect_;
}

}  // end namespace measurement_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(measurement_server::MeasurementServer)
