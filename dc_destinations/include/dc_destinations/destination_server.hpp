#ifndef DC_DESTINATIONS__DESTINATION_SERVER_HPP_
#define DC_DESTINATIONS__DESTINATION_SERVER_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "dc_core/destination.hpp"
#include "dc_util/filesystem_utils.hpp"
#include "dc_util/node_utils.hpp"
#include "dc_util/string_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <fluent-bit.h>
#pragma GCC diagnostic pop

namespace destination_server
{
using json = nlohmann::json;
using namespace std::chrono_literals;

/**
 * @class dc_destination::DestinationNode
 * @brief An server hosting a map of destination plugins
 */
class DestinationServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for dc_destination::DestinationNode
   * @param options Additional options to control creation of the node.
   */
  explicit DestinationServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DestinationServer() override;

  /**
   * @brief Loads destination plugins from parameter file
   * @return bool if successfully loaded the plugins
   */
  bool loadDestinationPlugins();

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

  // Plugins
  pluginlib::ClassLoader<dc_core::Destination> plugin_loader_;
  std::vector<pluginlib::UniquePtr<dc_core::Destination>> destinations_;
  std::vector<std::string> default_ids_;

  std::vector<std::string> destination_ids_;
  std::vector<std::string> ros_topics_;
  std::vector<std::vector<std::string>> destination_inputs_;
  std::vector<bool> destination_debug_;
  std::vector<std::string> destination_types_;
  std::vector<std::string> destination_time_format_;
  std::vector<std::string> destination_time_key_;
  std::vector<std::string> custom_str_params_list_;
  json custom_params_;

  // Fluent Bit
  flb_ctx_t* ctx_;
  void initFlb();
  void initFlbInputPlugin();
  void startFlbEngine();
  int flb_flush_, flb_grace_, flb_scheduler_cap_, flb_scheduler_base_;
  std::string flb_log_level_, flb_storage_path_, flb_storage_sync_, flb_storage_checksum_,
      flb_storage_backlog_mem_limit_, flb_input_tag_, flb_in_storage_type_, flb_in_storage_pause_on_chunks_overlimit_;
  bool flb_http_server_;
  std::string flb_http_listen_;
  int flb_http_port_;
  std::string ros2_plugin_path_;
  std::string ros2_plugin_path_default_;
  int ros2_plugin_spin_time_ms_;
};

}  // namespace destination_server

#endif  // DC_DESTINATIONS__DESTINATION_SERVER_HPP_
