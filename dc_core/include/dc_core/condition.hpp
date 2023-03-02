#ifndef DC_CORE_CONDITION_HPP_
#define DC_CORE_CONDITION_HPP_

#include <memory>
#include <string>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace dc_core
{
class Condition
{
public:
  using Ptr = std::shared_ptr<Condition>;

  /**
   * @brief Virtual destructor
   */
  virtual ~Condition()
  {
  }

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this condition
   * @param  tf A pointer to a TF buffer
   */
  virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to activate Condition and any threads involved in execution.
   */
  virtual void activate() = 0;

  virtual bool getState(dc_interfaces::msg::StringStamped msg) = 0;

  /**
   * @brief Method to deactivate Condition and any threads involved in execution.
   */
  virtual void deactivate() = 0;
};
}  // namespace dc_core

#endif  // DC_CORE_CONDITION_HPP_
