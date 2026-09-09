// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_CORE_MEASUREMENT_HPP_
#define DC_CORE_MEASUREMENT_HPP_

#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "dc_core/condition.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace dc_core
{

// Every setting handed to a Measurement's configure() beyond its node, name, Conditions and TF
// buffer. One field per setting, so a caller can't swap two same-typed arguments unnoticed.
struct MeasurementConfig
{
  // Plugin identification and output
  std::string measurement_plugin;  ///< Name of the plugin
  std::string group_key;           ///< Key the Group node merges Records of several Measurements by
  std::string topic_output;        ///< Topic where the result is published
  int polling_interval{ 1000 };    ///< Interval to which data is collected in milliseconds
  bool debug{ false };             ///< Print debug lines

  // Validation and routing
  bool enable_validator{ true };  ///< Validate the data against a JSON schema
  std::string json_schema_path;   ///< Path to the JSON schema
  std::vector<std::string> tags;  ///< Used to match to destination

  // Collection gating and counters
  bool init_collect{ true };                    ///< Collect when the node starts instead of waiting for the
                                                ///< polling_interval time to pass
  int init_max_measurements{ 0 };               ///< Collect a maximum of n measurements when starting the node
                                                ///< (-1 = never, 0 = infinite)
  int condition_max_measurements{ 0 };          ///< Collect a maximum of n measurements when conditions are
                                                ///< activated (-1 = never, 0 = infinite)
  std::vector<std::string> if_all_conditions;   ///< Collect only if all conditions are activated
  std::vector<std::string> if_any_conditions;   ///< Collect if any conditions is activated
  std::vector<std::string> if_none_conditions;  ///< Collect only if all conditions are not activated
  std::string gate_condition;                   ///< Name of a Condition that must become true once before any
                                                ///< collection is published; empty disables gating. Once true, the
                                                ///< gate latches open permanently and the condition is no longer
                                                ///< consulted -- distinct from if_all/if_any/if_none, which are
                                                ///< re-evaluated on every collection

  // Record shaping
  bool include_measurement_name{ true };     ///< Include measurement name in the JSON
  bool include_measurement_plugin{ false };  ///< Include measurement plugin name in the JSON
  std::vector<std::string> remote_keys;      ///< Destination names a remote path is computed for
  std::vector<std::string> remote_prefixes;  ///< Prefixes paired positionally with remote_keys,
                                             ///< prepended under all_base_path
  bool nested{ false };                      ///< Nest the collected JSON under the measurement name
  bool flatten{ false };                     ///< Flatten the collected JSON

  // Server-level settings, filled once by the MeasurementServer for every Measurement
  std::string save_local_base_path;           ///< Unexpanded base path Files are saved under
  std::string save_local_base_path_expanded;  ///< Same, with env vars and custom keys expanded
  std::string all_base_path;                  ///< Unexpanded base path of everything collected
  std::string all_base_path_expanded;         ///< Same, with env vars and custom keys expanded
  std::string run_id;                         ///< Unique ID of the current run
  bool run_id_enabled{ true };                ///< Whether the run ID is added to Records
  std::vector<nlohmann::json> custom_keys;    ///< Vector of JSON with custom keys

  // Incident capture
  double buffer_duration_sec{ 0.0 };     ///< Seconds of history to buffer instead of publishing live;
                                         ///< 0 (the default) disables buffering and preserves normal
                                         ///< live publishing (#287)
  double post_roll_duration_sec{ 0.0 };  ///< Seconds to keep publishing live after a flush, still
                                         ///< tagged with the same incident_id; 0 (the default)
                                         ///< means pre-roll only (#288)
  double cooldown_sec{ 0.0 };            ///< Seconds to ignore further FlushEvents once post-roll ends,
                                         ///< before buffering re-arms itself; 0 (the default) re-arms
                                         ///< immediately (#288)
  double max_flush_rate_hz{ 0.0 };       ///< Ceiling on how fast the buffered window is emitted once a
                                         ///< flush releases it, protecting Bridge/network bandwidth; 0
                                         ///< (the default) releases it in one burst (#289)
  std::string flush_topic;               ///< Topic to receive the FlushEvent that releases the buffered window,
                                         ///< tagging each released Record with the event's incident_id
};

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
   * @param  config Every other setting of this Measurement, one field each -- see MeasurementConfig
   */
  virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                         const std::map<std::string, std::shared_ptr<dc_core::Condition>>& conditions,
                         std::shared_ptr<tf2_ros::Buffer> tf, const MeasurementConfig& config) = 0;

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
