#include "dc_triggers/plugins/edge_trigger.hpp"

namespace dc_triggers
{

EdgeTrigger::EdgeTrigger() : dc_triggers::Trigger()
{
}

EdgeTrigger::~EdgeTrigger() = default;

bool EdgeTrigger::checkFired()
{
  if (!enabled_)
  {
    return false;
  }

  const bool satisfied = condition_set_.isSatisfied(dc_core::ConditionStateLookup{
      [this](const std::string& condition_name) { return getConditionState(condition_name); } });

  const bool fired = satisfied && !previously_satisfied_;
  previously_satisfied_ = satisfied;

  if (fired)
  {
    RCLCPP_INFO(logger_, "Trigger %s fired", trigger_name_.c_str());
  }

  return fired;
}

}  // namespace dc_triggers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_triggers::EdgeTrigger, dc_core::Trigger)
