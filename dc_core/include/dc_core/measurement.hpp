#ifndef DC_CORE_MEASUREMENT_HPP_
#define DC_CORE_MEASUREMENT_HPP_

#include <memory>
#include <string>

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
   * @param  timer_cb_group Callback group, common to all measurements
   * @param  tags Used to match to destination
   * @param  init_collect Collect when the node starts instead of waiting the first tick
   * @param  init_max_measurements Collect a maximum of n measurements when starting the node (-1 = never, 0 = infinite)
   * @param  include_measurement_name Include measurement name in the JSON
   * @param  if_all_conditions Collect only if all conditions are activated
   * @param  if_any_conditions Collect if any conditions is activated
   * @param  if_none_conditions Collect only if all conditions are not activated
   */
  virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                         std::shared_ptr<tf2_ros::Buffer> tf, const std::string& measurement_plugin,
                         const std::string& group_key, const std::string& topic_output, const int& polling_interval,
                         const bool& debug, const bool& enable_validator, const std::string& json_schema_path,
                         const std::vector<std::string>& tags, const bool& init_collect,
                         const int& init_max_measurements, const bool& include_measurement_name,
                         const std::vector<std::string>& if_all_conditions,
                         const std::vector<std::string>& if_any_conditions,
                         const std::vector<std::string>& if_none_conditions,
                         const rclcpp::CallbackGroup::SharedPtr& timer_cb_group) = 0;

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
