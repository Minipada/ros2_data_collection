// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_TRIGGERS__TRIGGER_HPP_
#define DC_TRIGGERS__TRIGGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "dc_core/condition.hpp"
#include "dc_core/condition_set.hpp"
#include "dc_core/trigger.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_triggers
{

/**
 * @class dc_triggers::Trigger
 * @brief Common plugin plumbing shared by every concrete Trigger, the way dc_conditions::Condition
 * is shared by every concrete Condition: owns the node handle, the Condition map the Trigger was
 * configured with, and the ConditionSet composing them, leaving checkFired() as the one thing a
 * concrete Trigger (e.g. EdgeTrigger) has to implement.
 */
class Trigger : public dc_core::Trigger
{
public:
  Trigger()
  {
  }

  ~Trigger() override = default;

  // an opportunity for derived classes to do something on configuration, if they choose
  virtual void onConfigure()
  {
  }

  // an opportunity for derived classes to do something on cleanup, if they choose
  virtual void onCleanup()
  {
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> getNode()
  {
    auto node = node_.lock();
    if (!node)
    {
      throw std::runtime_error{ "Failed to lock node" };
    }
    return node;
  }

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                 const std::map<std::string, std::shared_ptr<dc_core::Condition>>& conditions,
                 const std::vector<std::string>& if_all_conditions, const std::vector<std::string>& if_any_conditions,
                 const std::vector<std::string>& if_none_conditions) override
  {
    node_ = parent;
    auto node = node_.lock();

    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    trigger_name_ = name;
    conditions_ = conditions;
    condition_set_ = dc_core::ConditionSet(if_all_conditions, if_any_conditions, if_none_conditions);

    RCLCPP_INFO(logger_, "Done configuring %s", trigger_name_.c_str());

    onConfigure();
  }

  void cleanup() override
  {
    onCleanup();
  }

  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating trigger %s", trigger_name_.c_str());
    enabled_ = true;
  }

  void deactivate() override
  {
    enabled_ = false;
  }

protected:
  // Current state of one of the configured Conditions. A name that is not a configured Condition
  // reads as false rather than dereferencing a null plugin -- mirrors
  // dc_measurements::Measurement::getConditionState().
  bool getConditionState(const std::string& condition_name)
  {
    auto condition_it = conditions_.find(condition_name);
    if (condition_it == conditions_.end() || !condition_it->second)
    {
      RCLCPP_ERROR_STREAM(logger_, "Trigger " << trigger_name_ << ": '" << condition_name
                                              << "' is not a configured condition; treating it as false.");
      return false;
    }
    // Triggers have no data sample of their own to hand a Condition the way a Measurement does --
    // only Conditions that maintain their own state (e.g. subscribing to a topic directly, such
    // as Moving) are meaningful composed into a Trigger.
    return condition_it->second->getState(dc_interfaces::msg::StringStamped());
  }

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string trigger_name_;
  bool enabled_{ false };

  std::map<std::string, std::shared_ptr<dc_core::Condition>> conditions_;
  dc_core::ConditionSet condition_set_;

  rclcpp::Logger logger_{ rclcpp::get_logger("dc_triggers") };
};

}  // namespace dc_triggers

#endif  // DC_TRIGGERS__TRIGGER_HPP_
