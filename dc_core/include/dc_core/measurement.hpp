#ifndef DC_CORE_MEASUREMENT_HPP_
#define DC_CORE_MEASUREMENT_HPP_

#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "dc_core/condition.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace dc_core
{
class Measurement
{
public:
  using Ptr = std::shared_ptr<Measurement>;
  using json = nlohmann::json;

  /**
   * @brief Virtual destructor
   */
  virtual ~Measurement()
  {
  }

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this measurement
   * @param  tf A pointer to a TF buffer
   * @param  measurement_plugin The name of the plugin
   * @param  topic_output The topic where result will be published
   * @param  polling_interval Interval to which data is collected in milliseconds
   * @param  debug Print debug lines
   * @param  enable_validator Will validate the data against a JSON schema
   * @param  json_schema_path Path to the JSON schema
   * @param  tags Used to match to destination
   * @param  init_collect Collect when the node starts instead of waiting for the polling_interval time to pass
   * @param  init_max_measurements Collect a maximum of n measurements when starting the node (-1 = never, 0 = infinite)
   * @param  include_measurement_name Include measurement name in the JSON
   * @param  include_measurement_plugin Include measurement plugin name in the JSON
   * @param  condition_max_measurements Collect a maximum of n measurements when conditions are activated (-1 = never, 0
   * = infinite)
   * @param  if_all_conditions Collect only if all conditions are activated
   * @param  if_any_conditions Collect if any conditions is activated
   * @param  if_none_conditions Collect only if all conditions are not activated
   * @param  gate_condition Name of a Condition that must become true once before any collection is published; empty
   * disables gating. Once true, the gate latches open permanently and the condition is no longer consulted -- distinct
   * from if_all/if_any/if_none, which are re-evaluated on every collection
   * @param  run_id Unique ID of the current run
   * @param  run_id Whether Run Id is enabled
   * @param  custom_keys Vector of JSON with custom keys
   * @param  buffer_duration_sec Seconds of history to buffer instead of publishing live; 0 (the
   * default) disables buffering and preserves normal live publishing (#287)
   * @param  post_roll_duration_sec Seconds to keep publishing live after a flush, still tagged
   * with the same incident_id; 0 (the default) means pre-roll only (#288)
   * @param  cooldown_sec Seconds to ignore further FlushEvents once post-roll ends, before
   * buffering re-arms itself; 0 (the default) re-arms immediately (#288)
   * @param  max_flush_rate_hz Ceiling on how fast the buffered window is emitted once a flush
   * releases it, protecting Bridge/network bandwidth; 0 (the default) releases it in one burst
   * (#289)
   * @param  flush_topic Topic to receive the FlushEvent that releases the buffered window,
   * tagging each released Record with the event's incident_id
   */
  virtual void
  configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
            const std::map<std::string, std::shared_ptr<dc_core::Condition>>& conditions,
            std::shared_ptr<tf2_ros::Buffer> tf, const std::string& measurement_plugin, const std::string& group_key,
            const std::string& topic_output, const int& polling_interval, const bool& debug,
            const bool& enable_validator, const std::string& json_schema_path, const std::vector<std::string>& tags,
            const bool& init_collect, const int& init_max_measurements, const bool& include_measurement_name,
            const bool& include_measurement_plugin, const int& condition_max_measurements,
            const std::vector<std::string>& if_all_conditions, const std::vector<std::string>& if_any_conditions,
            const std::vector<std::string>& if_none_conditions, const std::string& gate_condition,
            const std::vector<std::string>& remote_keys, const std::vector<std::string>& remote_prefixes,
            const bool& nest, const bool& flatten, const std::string& save_local_base_path,
            const std::string& all_base_path, const std::string& all_base_path_expanded,
            const std::string& save_local_base_path_expanded, const std::string& run_id, const bool& run_id_enabled,
            const std::vector<json>& custom_keys, const double& buffer_duration_sec,
            const double& post_roll_duration_sec, const double& cooldown_sec, const double& max_flush_rate_hz,
            const std::string& flush_topic) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to activate Measurement and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactivate Measurement and any threads involved in execution.
   */
  virtual void deactivate() = 0;
};
}  // namespace dc_core

#endif  // DC_CORE_MEASUREMENT_HPP_
