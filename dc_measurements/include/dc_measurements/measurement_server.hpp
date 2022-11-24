#ifndef DC_MEASUREMENTS__MEASUREMENT_SERVER_HPP_
#define DC_MEASUREMENTS__MEASUREMENT_SERVER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include "dc_core/measurement.hpp"
#include "dc_util/node_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace measurement_server
{

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
  explicit MeasurementServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MeasurementServer();

  /**
   * @brief Loads behavior plugins from parameter file
   * @return bool if successfully loaded the plugins
   */
  bool loadMeasurementPlugins();

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

  void setBaseSavePath();

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  // Plugins
  pluginlib::ClassLoader<dc_core::Measurement> plugin_loader_;
  std::vector<pluginlib::UniquePtr<dc_core::Measurement>> measurements_;

  std::vector<std::string> measurement_plugins_;
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

  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
};

}  // namespace measurement_server

#endif  // DC_MEASUREMENTS__MEASUREMENT_SERVER_HPP_
