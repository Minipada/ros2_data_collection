#ifndef DC_MEASUREMENTS__MEASUREMENT_SERVER_HPP_
#define DC_MEASUREMENTS__MEASUREMENT_SERVER_HPP_

#include <uuid/uuid.h>

#include <boost/algorithm/string.hpp>
#include <nlohmann/json.hpp>

#include "dc_core/condition.hpp"
#include "dc_core/measurement.hpp"
#include "dc_util/filesystem_utils.hpp"
#include "dc_util/node_utils.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace measurement_server
{
using json = nlohmann::json;

/**
 * @class dc_measurement::MeasurementNode
 * @brief An server hosting a map of behavior plugins
 */
class MeasurementServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for dc_measurement::MeasurementNode
   * @param options Additional options to control creation of the node.
   */
  explicit MeasurementServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                             const std::vector<std::string>& measurement_plugins = std::vector<std::string>());
  ~MeasurementServer() override;

  /**
   * @brief Loads measurement plugins from parameter file
   * @return bool if successfully loaded the plugins
   */
  bool loadMeasurementPlugins();

  /**
   * @brief Loads condition plugins from parameter file
   * @return bool if successfully loaded the plugins
   */
  bool loadConditionPlugins();

  std::vector<std::string> getMeasurementPlugins();
  std::vector<std::string> getMeasurementTypes();
  std::vector<std::string> getMeasurementGroupKeys();
  std::vector<std::string> getMeasurementTopicOutput();
  std::vector<int> getMeasurementPollingInterval();
  std::vector<bool> getMeasurementInitCollect();

protected:
  /**
   * @brief Configure lifecycle server
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Activate lifecycle server
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Deactivate lifecycle server
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Cleanup lifecycle server
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Shutdown lifecycle server
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  void setCustomParameters();
  void setRunId();
  void setBaseSavePath();

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Measurements
  pluginlib::ClassLoader<dc_core::Measurement> measurement_plugin_loader_;
  std::vector<pluginlib::UniquePtr<dc_core::Measurement>> measurements_;
  // Run ID
  json custom_params_;
  std::string run_id_;
  std::string run_id_counter_path_;
  bool run_id_enabled_;
  bool run_id_counter_;
  bool run_id_uuid_;

  // std::vector<std::string> measurement_plugins_;
  std::vector<std::string> measurement_group_key_;
  std::vector<std::string> measurement_ids_;
  std::vector<int> measurement_polling_interval_;
  std::vector<std::string> measurement_topic_outputs_;
  std::vector<std::string> measurement_types_;
  std::vector<bool> measurement_debug_;
  std::vector<bool> measurement_enable_validator_;
  std::vector<std::string> measurement_json_schema_path_;
  std::vector<std::vector<std::string>> measurement_tags_;
  std::vector<bool> measurement_init_collect_;
  std::vector<int> measurement_init_max_measurements_;
  std::vector<bool> measurement_include_measurement_name_;
  std::vector<bool> measurement_include_measurement_plugin_;
  std::vector<std::vector<std::string>> measurement_if_all_conditions_;
  std::vector<std::vector<std::string>> measurement_if_any_conditions_;
  std::vector<std::vector<std::string>> measurement_if_none_conditions_;
  std::vector<std::vector<std::string>> measurement_remote_keys_;
  std::vector<std::vector<std::string>> measurement_remote_prefixes_;
  std::string save_local_base_path_;
  std::string save_local_base_path_expanded_;
  std::string all_base_path_;
  std::string all_base_path_expanded_;
  std::vector<std::string> measurement_custom_str_params_;
  std::map<std::string, std::string> custom_str_params_map_;

  // Conditions
  std::map<std::string, std::shared_ptr<dc_core::Condition>> conditions_;
  std::vector<std::string> condition_types_;
  pluginlib::ClassLoader<dc_core::Condition> condition_plugin_loader_;
  std::vector<std::string> condition_ids_;
  std::vector<int> measurement_condition_max_measurements_;
};

}  // namespace measurement_server

#endif  // DC_MEASUREMENTS__MEASUREMENT_SERVER_HPP_
