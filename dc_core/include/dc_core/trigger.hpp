#ifndef DC_CORE_TRIGGER_HPP_
#define DC_CORE_TRIGGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "dc_core/condition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace dc_core
{
/**
 * @class dc_core::Trigger
 * @brief Abstract plugin base for Triggers, distinct from Condition: a Condition gates whether a
 * Measurement's Records are collected, while a Trigger fires a one-shot signal (see
 * dc_interfaces::msg::FlushEvent) that a Trigger broadcast node publishes for downstream
 * Measurements to react to. Lifecycle methods mirror dc_core::Condition's exactly, so both plugin
 * families stay familiar to write and to load.
 */
class Trigger
{
public:
  using Ptr = std::shared_ptr<Trigger>;

  /**
   * @brief Virtual destructor
   */
  virtual ~Trigger()
  {
  }

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this trigger
   * @param  conditions Every Condition plugin loaded by the parent node, keyed by name, for this
   * Trigger to compose via if_all/if_any/if_none
   * @param  if_all_conditions Fire only once every named Condition is active
   * @param  if_any_conditions Fire once any named Condition is active
   * @param  if_none_conditions Fire only once no named Condition is active
   */
  virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                         const std::map<std::string, std::shared_ptr<dc_core::Condition>>& conditions,
                         const std::vector<std::string>& if_all_conditions,
                         const std::vector<std::string>& if_any_conditions,
                         const std::vector<std::string>& if_none_conditions) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to activate Trigger and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactivate Trigger and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Evaluate the composed Condition state now. Called on a poll cadence by the owning
   * node; returns true at most once per firing (e.g. EdgeTrigger returns true exactly once per
   * false->true rising edge, not on every poll while the composed result stays true).
   */
  virtual bool checkFired() = 0;
};
}  // namespace dc_core

#endif  // DC_CORE_TRIGGER_HPP_
