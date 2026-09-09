// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/measurement_server.hpp"

#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "dc_measurements/measurement_config.hpp"

namespace measurement_server
{

using namespace std::chrono_literals;  // NOLINT

namespace
{

// robot_name is the one custom key whose identity a fleet can't hand-edit per robot (#442):
// literal value, then a file's contents, then the machine's hostname as the default.
constexpr const char* kRobotNameKey = "robot_name";

std::string resolveRobotNameFromHostname()
{
  char hostname_buf[256] = { 0 };
  if (gethostname(hostname_buf, sizeof(hostname_buf) - 1) != 0)
  {
    throw std::runtime_error(std::string("robot_name: failed to resolve hostname: ") + std::strerror(errno));
  }
  return std::string(hostname_buf);
}

std::string resolveRobotNameFromFile(const std::string& path)
{
  std::ifstream ifs(path);
  if (!ifs.good())
  {
    throw std::runtime_error("robot_name: could not read value_from_file '" + path + "'");
  }
  return dc_util::get_file_content(path);
}

std::string resolveRobotName(const std::string& value, const std::string& value_from_file)
{
  std::string resolved;
  std::string source;

  if (!value.empty())
  {
    resolved = value;
    source = "the literal value";
  }
  else if (!value_from_file.empty())
  {
    resolved = resolveRobotNameFromFile(value_from_file);
    source = "value_from_file '" + value_from_file + "'";
  }
  else
  {
    resolved = resolveRobotNameFromHostname();
    source = "the hostname";
  }

  if (resolved.empty())
  {
    throw std::runtime_error("robot_name: resolved to an empty value from " + source);
  }

  return resolved;
}

}  // namespace

MeasurementServer::MeasurementServer(const rclcpp::NodeOptions& options,
                                     const std::vector<std::string>& measurement_plugins)
  : nav2_util::LifecycleNode("measurement_server", "", options)
  , measurement_plugin_loader_("dc_core", "dc_core::Measurement")
  , condition_plugin_loader_("dc_core", "dc_core::Condition")
{
  measurement_ids_ = dc_util::get_str_array_param(this, "measurement_plugins", measurement_plugins);
  condition_ids_ = dc_util::get_str_array_param(this, "condition_plugins", std::vector<std::string>());
}

void MeasurementServer::setBaseSavePath()
{
  save_local_base_path_ = dc_util::get_str_param(this, "save_local_base_path", "$HOME/ros2/data/%Y/%M/%D/%H");
  all_base_path_ = dc_util::get_str_param(this, "all_base_path", "");

  save_local_base_path_expanded_ = dc_util::expand_env(save_local_base_path_);
  save_local_base_path_expanded_ = dc_util::expand_values(save_local_base_path_expanded_, this);
  RCLCPP_INFO(get_logger(), "Base save path expanded to %s", save_local_base_path_expanded_.c_str());
  all_base_path_expanded_ = dc_util::expand_env(all_base_path_);
  all_base_path_expanded_ = dc_util::expand_values(all_base_path_expanded_, this, "custom_keys_str.", ".value");
  RCLCPP_INFO(get_logger(), "All Base path expanded to %s", all_base_path_expanded_.c_str());
}

void MeasurementServer::setCustomKeys()
{
  // Declared for override parity with the per-parameter custom_keys_str.<name>.force_override
  // below; not consumed here (each custom key's own force_override is what's actually read).
  dc_util::get_bool_type_param(this, "custom_keys_str", "force_override", false);
  custom_key_str_list_ = dc_util::get_str_array_param(this, "custom_key_str_list", std::vector<std::string>());

  for (auto param = std::begin(measurement_custom_keys_str_); param != std::end(measurement_custom_keys_str_); ++param)
  {
    custom_keys_str_map_[*param] = dc_util::get_str_param(this, *param, "");
  }

  custom_keys_.resize(custom_key_str_list_.size());

  for (size_t i = 0; i < custom_key_str_list_.size(); i++)
  {
    auto custom_key = custom_key_str_list_[i];
    std::string custom_key_ns = "custom_keys_str." + custom_key;
    std::string key = dc_util::get_str_type_param(this, custom_key_ns, "name", "");
    std::string value = dc_util::get_str_type_param(this, custom_key_ns, "value", "");
    std::string value_from_file = dc_util::get_str_type_param(this, custom_key_ns, "value_from_file", "");
    bool force_override = dc_util::get_bool_type_param(this, custom_key_ns, "force_override", false);

    custom_keys_[i]["key"] = key;
    custom_keys_[i]["override"] = force_override;
    if (custom_key == kRobotNameKey)
    {
      custom_keys_[i]["value"] = resolveRobotName(value, value_from_file);
    }
    else if (!value.empty())
    {
      custom_keys_[i]["value"] = value;
    }
    else if (!value_from_file.empty())
    {
      custom_keys_[i]["value"] = dc_util::get_file_content(value_from_file);
    }
  }
}

void MeasurementServer::setRunId()
{
  auto node = shared_from_this();
  run_id_enabled_ = dc_util::get_bool_type_param(node, "run_id", "enabled", true);
  run_id_counter_ = dc_util::get_bool_type_param(node, "run_id", "counter", true);
  run_id_counter_path_ =
      dc_util::expand_env(dc_util::get_str_type_param(node, "run_id", "counter_path", "$HOME/run_id"));
  run_id_uuid_ = dc_util::get_bool_type_param(node, "run_id", "uuid", false);

  if (run_id_enabled_)
  {
    if (run_id_uuid_ && run_id_counter_)
    {
      throw std::runtime_error("Please select only one source for run ID");
    }
    else if (run_id_uuid_)
    {
      uuid_t uuid_obj;
      char uuid_str[100];
      uuid_generate(uuid_obj);
      uuid_unparse(uuid_obj, uuid_str);
      run_id_ = uuid_str;
    }
    else if (run_id_counter_)
    {
      // Check file exists, if not create it
      if (!std::ifstream(run_id_counter_path_))
      {
        // Create parent directory
        auto parent_dir = std::filesystem::path(run_id_counter_path_).parent_path().u8string();
        std::filesystem::create_directories(parent_dir);
        // Create file
        std::ofstream file(run_id_counter_path_);
        if (!file)
        {
          throw std::runtime_error("UUID counter file could not be created");
        }
        else
        {
          run_id_ = "1";
          dc_util::write_str_file(run_id_counter_path_, "1");
        }
      }
      // If counter file exists
      else
      {
        // Get new run_id
        run_id_ = std::to_string(stoi(dc_util::get_file_content(run_id_counter_path_)) + 1);
        dc_util::write_str_file(run_id_counter_path_, run_id_);
      }
    }
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
  setRunId();
  setCustomKeys();
  setBaseSavePath();

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

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
    dc_core::MeasurementConfig config = dc_measurements::read_measurement_config(node, measurement_ids_[i]);

    // Server-level settings, shared by every Measurement
    config.save_local_base_path = save_local_base_path_;
    config.save_local_base_path_expanded = save_local_base_path_expanded_;
    config.all_base_path = all_base_path_;
    config.all_base_path_expanded = all_base_path_expanded_;
    config.run_id = run_id_;
    config.run_id_enabled = run_id_enabled_;
    config.custom_keys = custom_keys_;

    try
    {
      RCLCPP_INFO_STREAM(
          get_logger(),
          "Creating measurement plugin "
              << measurement_ids_[i].c_str() << ": Type " << config.measurement_plugin.c_str()
              << ", Group key: " << config.group_key << ", Polling interval: " << config.polling_interval
              << ", Debug: " << (int)config.debug << ", Validator enabled: " << (int)config.enable_validator
              << ", Schema path: " << config.json_schema_path.c_str() << ", Tags: [" << dc_util::join(config.tags, ",")
              << "], Init collect: " << (int)config.init_collect << ", Init Max measurement: "
              << config.init_max_measurements << ", Include measurement name: " << config.include_measurement_name
              << ", Include measurement plugin name: " << config.include_measurement_plugin << ", Remote keys: "
              << dc_util::join(config.remote_keys) << ", Remote prefixes: " << dc_util::join(config.remote_prefixes)
              << ", Nest: " << (int)config.nested << ", Flatten: " << (int)config.flatten
              << ", Include measurement plugin name: " << config.include_measurement_plugin
              << ", Max measurement on condition: " << config.condition_max_measurements
              << ", If all condition: " << dc_util::join(config.if_all_conditions, ",")
              << ", If any condition: " << dc_util::join(config.if_any_conditions, ",") << ", If none condition: "
              << dc_util::join(config.if_none_conditions, ",") << ", Gate condition: " << config.gate_condition
              << ", Buffer duration sec: " << config.buffer_duration_sec << ", Post roll duration sec: "
              << config.post_roll_duration_sec << ", Cooldown sec: " << config.cooldown_sec
              << ", Max flush rate hz: " << config.max_flush_rate_hz << ", Flush topic: " << config.flush_topic);

      measurements_.push_back(measurement_plugin_loader_.createUniqueInstance(config.measurement_plugin));
      measurements_.back()->configure(node, measurement_ids_[i], conditions_, tf_, config);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(get_logger(),
                   "Failed to create measurement %s of type %s."
                   " Exception: %s",
                   measurement_ids_[i].c_str(), config.measurement_plugin.c_str(), ex.what());
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

}  // end namespace measurement_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(measurement_server::MeasurementServer)
