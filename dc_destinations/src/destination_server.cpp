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

  // Fluent Bit
  // https://docs.fluentbit.io/manual/administration/configuring-fluent-bit/classic-mode/configuration-file#config_section
  nav2_util::declare_parameter_if_not_declared(this, "flb.flush", rclcpp::ParameterValue(1));
  nav2_util::declare_parameter_if_not_declared(this, "flb.grace", rclcpp::ParameterValue(1));
  nav2_util::declare_parameter_if_not_declared(this, "flb.log_level", rclcpp::ParameterValue("info"));
  nav2_util::declare_parameter_if_not_declared(this, "flb.storage_path",
                                               rclcpp::ParameterValue("/var/log/flb-storage/"));
  nav2_util::declare_parameter_if_not_declared(this, "flb.storage_sync", rclcpp::ParameterValue("normal"));
  nav2_util::declare_parameter_if_not_declared(this, "flb.storage_checksum", rclcpp::ParameterValue("off"));
  nav2_util::declare_parameter_if_not_declared(this, "flb.storage_backlog_mem_limit", rclcpp::ParameterValue("5M"));
  nav2_util::declare_parameter_if_not_declared(this, "flb.scheduler_cap", rclcpp::ParameterValue(2000));
  nav2_util::declare_parameter_if_not_declared(this, "flb.scheduler_base", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(this, "flb.http_server", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(this, "flb.http_listen", rclcpp::ParameterValue("0.0.0.0"));
  nav2_util::declare_parameter_if_not_declared(this, "flb.http_port", rclcpp::ParameterValue(2020));
  nav2_util::declare_parameter_if_not_declared(this, "flb.in_storage_type", rclcpp::ParameterValue("filesystem"));
  nav2_util::declare_parameter_if_not_declared(this, "flb.in_storage_pause_on_chunks_overlimit",
                                               rclcpp::ParameterValue("off"));

  std::string package_directory = ament_index_cpp::get_package_prefix("fluent_bit_plugins");
  ros2_plugin_path_default_ = package_directory + "/lib/flb-in_ros2.so";
  nav2_util::declare_parameter_if_not_declared(this, "ros2_plugin_path",
                                               rclcpp::ParameterValue(ros2_plugin_path_default_));
  nav2_util::declare_parameter_if_not_declared(this, "ros2_plugin_spin_time_ms", rclcpp::ParameterValue(100));

  nav2_util::declare_parameter_if_not_declared(this, "run_id.enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(this, "run_id.counter", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(this, "run_id.counter_path", rclcpp::ParameterValue("$HOME/run_id"));
  nav2_util::declare_parameter_if_not_declared(this, "run_id.uuid", rclcpp::ParameterValue(false));

  flb_flush_ = this->get_parameter("flb.flush").as_int();
  flb_grace_ = this->get_parameter("flb.grace").as_int();
  flb_log_level_ = this->get_parameter("flb.log_level").as_string();
  flb_storage_path_ = this->get_parameter("flb.storage_path").as_string();
  flb_storage_sync_ = this->get_parameter("flb.storage_sync").as_string();
  flb_storage_checksum_ = this->get_parameter("flb.storage_checksum").as_string();
  flb_storage_backlog_mem_limit_ = this->get_parameter("flb.storage_backlog_mem_limit").as_string();
  flb_scheduler_cap_ = this->get_parameter("flb.scheduler_cap").as_int();
  flb_scheduler_base_ = this->get_parameter("flb.scheduler_base").as_int();
  flb_http_server_ = this->get_parameter("flb.http_server").as_bool();
  flb_http_listen_ = this->get_parameter("flb.http_listen").as_string();
  flb_http_port_ = this->get_parameter("flb.http_port").as_int();
  flb_in_storage_type_ = this->get_parameter("flb.in_storage_type").as_string();
  flb_in_storage_pause_on_chunks_overlimit_ =
      this->get_parameter("flb.in_storage_pause_on_chunks_overlimit").as_string();
  nav2_util::declare_parameter_if_not_declared(this, "custom_str_params_list",
                                               rclcpp::ParameterValue(std::vector<std::string>()));
  ros2_plugin_path_ = this->get_parameter("ros2_plugin_path").as_string();
  ros2_plugin_spin_time_ms_ = this->get_parameter("ros2_plugin_spin_time_ms").as_int();

  custom_str_params_list_ = this->get_parameter("custom_str_params_list").as_string_array();

  run_id_enabled_ = this->get_parameter("run_id.enabled").as_bool();
  run_id_counter_ = this->get_parameter("run_id.counter").as_bool();
  run_id_counter_path_ = dc_util::expand_env(this->get_parameter("run_id.counter_path").as_string());
  run_id_uuid_ = this->get_parameter("run_id.uuid").as_bool();

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

  for (auto& custom_param : custom_str_params_list_)
  {
    nav2_util::declare_parameter_if_not_declared(this, "custom_str_params." + custom_param + ".name",
                                                 rclcpp::ParameterValue(""));
    nav2_util::declare_parameter_if_not_declared(this, "custom_str_params." + custom_param + ".value",
                                                 rclcpp::ParameterValue(""));
    nav2_util::declare_parameter_if_not_declared(this, "custom_str_params." + custom_param + ".value_from_file",
                                                 rclcpp::ParameterValue(""));
    std::string key = this->get_parameter("custom_str_params." + custom_param + ".name").as_string();
    std::string value = this->get_parameter("custom_str_params." + custom_param + ".value").as_string();
    std::string value_from_file =
        this->get_parameter("custom_str_params." + custom_param + ".value_from_file").as_string();

    if (!value.empty())
    {
      custom_params_[key] = value;
    }
    else if (!value_from_file.empty())
    {
      custom_params_[key] = dc_util::get_file_content(value_from_file);
    }
  }
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
  destination_time_format_.resize(destination_ids_.size());
  destination_time_key_.resize(destination_ids_.size());

  initFlb();

  if (!loadDestinationPlugins())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  initFlbInputPlugin();
  startFlbEngine();

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
    destination_time_format_[i] = dc_util::get_str_type_param(node, destination_ids_[i], "time_format", "double");
    destination_time_key_[i] = dc_util::get_str_type_param(node, destination_ids_[i], "time_key", "date");

    try
    {
      RCLCPP_INFO_STREAM(get_logger(), "Creating destination plugin "
                                           << destination_ids_[i].c_str() << ": Type " << destination_types_[i].c_str()
                                           << ", Debug: " << (int)destination_debug_[i]
                                           << ", Time format: " << destination_time_format_[i]
                                           << ". Time key: " << destination_time_key_[i]);

      destinations_.push_back(plugin_loader_.createUniqueInstance(destination_types_[i]));
      destinations_.back()->configure(node, destination_ids_[i], destination_inputs_[i], ctx_, destination_debug_[i],
                                      flb_in_storage_type_, destination_time_format_[i], destination_time_key_[i],
                                      custom_params_, run_id_, run_id_enabled_);
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

void DestinationServer::initFlb()
{
  /* Create library context */
  mk_core_init();
  ctx_ = flb_create();

  /* Enable metrics if option enabled */
  std::string http_server = "";
  if (flb_http_server_)
  {
    http_server = "on";
  }
  else
  {
    http_server = "off";
  }

  flb_service_set(ctx_, "flush", std::to_string(flb_flush_).c_str(), "grace", std::to_string(flb_grace_).c_str(),
                  "log_Level", flb_log_level_.c_str(), "storage.path", flb_storage_path_.c_str(), "storage.sync",
                  flb_storage_sync_.c_str(), "storage.checksum", flb_storage_checksum_.c_str(),
                  "storage.backlog.mem_limit", flb_storage_backlog_mem_limit_.c_str(), "scheduler.cap",
                  std::to_string(flb_scheduler_cap_).c_str(), "scheduler.base",
                  std::to_string(flb_scheduler_base_).c_str(), "http_server", http_server.c_str(), "http_listen",
                  flb_http_listen_.c_str(), "http_port", std::to_string(flb_http_port_).c_str(), nullptr);

  if (!ctx_)
  {
    throw std::runtime_error("Cannot create Fluent Bit library context");
  }

  RCLCPP_INFO(get_logger(), "Fluent Bit service initialized");
}

void DestinationServer::startFlbEngine()
{
  /* Start the engine */
  RCLCPP_INFO(get_logger(), "Starting Flb engine...");
  int ret = flb_start(ctx_);
  if (ret == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot start Fluent Bit engine");
  }
  RCLCPP_INFO(get_logger(), "Started Flb engine");
}

void DestinationServer::initFlbInputPlugin()
{
  int in_ffd;
  int ret = 0;

  /* Enable the input plugin for manual data ingestion */
  RCLCPP_INFO(get_logger(), "Loading input ros2 shared library %s...", ros2_plugin_path_.c_str());
  if (flb_plugin_load_router(strdup(ros2_plugin_path_.c_str()), ctx_->config) != 0)
  {
    flb_error("[plugin] error loading c plugin: %s", ros2_plugin_path_.c_str());
    throw std::runtime_error("Cannot load plugin");
  }

  RCLCPP_INFO(get_logger(), "Loaded input ros2 shared library %s", ros2_plugin_path_.c_str());

  in_ffd = flb_input(ctx_, "ros2", nullptr);
  if (in_ffd == -1)
  {
    flb_destroy(ctx_);
    throw std::runtime_error("Cannot initialize Fluent Bit input ros2 plugin");
  }

  ros_topics_ = dc_util::remove_duplicates(dc_util::flatten(destination_inputs_));

  ret += flb_input_set(ctx_, in_ffd, "tag", "ros2", nullptr);

  ret += flb_input_set(ctx_, in_ffd, "topics", dc_util::to_space_separated_string(ros_topics_).c_str(), nullptr);
  ret += flb_input_set(ctx_, in_ffd, "spin_time", std::to_string(ros2_plugin_spin_time_ms_).c_str(), nullptr);
  ret += flb_input_set(ctx_, in_ffd, "storage.type", flb_in_storage_type_.c_str(), nullptr);
  ret += flb_input_set(ctx_, in_ffd, "storage.pause_on_chunks_overlimit",
                       flb_in_storage_pause_on_chunks_overlimit_.c_str(), nullptr);

  if (ret != 0)
  {
    throw std::runtime_error(std::string("Cannot initialize parameters Fluent Bit input ros2 plugin. topics: \"") +
                             dc_util::to_space_separated_string(ros_topics_) + "\", storage.type: \"" +
                             flb_in_storage_type_.c_str() + "\", storage.pause_on_chunks_overlimit: \"" +
                             flb_in_storage_pause_on_chunks_overlimit_.c_str() + "\"");
  }

  RCLCPP_INFO(get_logger(), "Flb ros2 plugin initialized. ret=%d", ret);
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
