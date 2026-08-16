#ifndef DC_TRIGGERS__TRIGGER_BROADCAST_NODE_HPP_
#define DC_TRIGGERS__TRIGGER_BROADCAST_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "dc_core/condition.hpp"
#include "dc_core/trigger.hpp"
#include "dc_interfaces/msg/flush_event.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"

namespace trigger_broadcast_node
{

/**
 * @class trigger_broadcast_node::TriggerBroadcastNode
 * @brief Loads the Condition plugins a Trigger composes plus one Trigger plugin, polls the
 * Trigger on a timer, and publishes a dc_interfaces::msg::FlushEvent -- minting a fresh
 * incident_id -- each time it fires. Mirrors measurement_server::MeasurementServer's plugin
 * loading, scaled down to the single Trigger this node hosts.
 */
class TriggerBroadcastNode : public nav2_util::LifecycleNode
{
public:
  explicit TriggerBroadcastNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~TriggerBroadcastNode() override;

  /**
   * @brief Loads every Condition plugin named in the condition_plugins parameter.
   * @return bool if successfully loaded the plugins
   */
  bool loadConditionPlugins();

  /**
   * @brief Loads the configured Trigger plugin.
   * @return bool if successfully loaded the plugin
   */
  bool loadTriggerPlugin();

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  // Polls the Trigger plugin; on a fire, mints an incident_id and publishes a FlushEvent.
  void checkTrigger();
  std::string mintIncidentId();

  // Conditions, loaded the same way measurement_server does.
  std::map<std::string, std::shared_ptr<dc_core::Condition>> conditions_;
  std::vector<std::string> condition_ids_;
  std::vector<std::string> condition_types_;
  pluginlib::ClassLoader<dc_core::Condition> condition_plugin_loader_;

  // The single Trigger this node hosts.
  const std::string trigger_id_{ "trigger" };
  std::string trigger_type_;
  std::vector<std::string> trigger_if_all_conditions_;
  std::vector<std::string> trigger_if_any_conditions_;
  std::vector<std::string> trigger_if_none_conditions_;
  std::string trigger_topic_;
  int trigger_polling_interval_;
  pluginlib::ClassLoader<dc_core::Trigger> trigger_plugin_loader_;
  pluginlib::UniquePtr<dc_core::Trigger> trigger_;

  rclcpp_lifecycle::LifecyclePublisher<dc_interfaces::msg::FlushEvent>::SharedPtr flush_pub_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
};

}  // namespace trigger_broadcast_node

#endif  // DC_TRIGGERS__TRIGGER_BROADCAST_NODE_HPP_
