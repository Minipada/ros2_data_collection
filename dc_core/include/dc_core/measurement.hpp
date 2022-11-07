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
   * @param  timer_cb_group Callback group, common to all measurements
   */
  virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                         std::shared_ptr<tf2_ros::Buffer> tf, const std::string& measurement_plugin,
                         const std::string& topic_output, const int& polling_interval, const bool& debug,
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
