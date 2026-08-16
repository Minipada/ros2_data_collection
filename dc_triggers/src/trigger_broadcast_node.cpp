#include "dc_triggers/trigger_broadcast_node.hpp"

#include <uuid/uuid.h>

#include "dc_util/node_utils.hpp"

namespace trigger_broadcast_node
{

TriggerBroadcastNode::TriggerBroadcastNode(const rclcpp::NodeOptions& options)
  : nav2_util::LifecycleNode("trigger_broadcast_node", "", options)
  , condition_plugin_loader_("dc_core", "dc_core::Condition")
  , trigger_plugin_loader_("dc_core", "dc_core::Trigger")
{
  condition_ids_ = dc_util::get_str_array_param(this, "condition_plugins", std::vector<std::string>());
}

TriggerBroadcastNode::~TriggerBroadcastNode()
{
  trigger_.reset();
}

bool TriggerBroadcastNode::loadConditionPlugins()
{
  auto node = shared_from_this();
  condition_types_.resize(condition_ids_.size());

  for (size_t i = 0; i != condition_ids_.size(); i++)
  {
    condition_types_[i] = dc_util::get_str_type_param(node, condition_ids_[i], "plugin");
    try
    {
      RCLCPP_INFO(get_logger(), "Creating condition plugin %s: Type %s", condition_ids_[i].c_str(),
                  condition_types_[i].c_str());
      conditions_[condition_ids_[i]] = condition_plugin_loader_.createUniqueInstance(condition_types_[i]);
      conditions_[condition_ids_[i]]->configure(node, condition_ids_[i]);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(get_logger(),
                   "Failed to create condition %s of type %s."
                   " Exception: %s",
                   condition_ids_[i].c_str(), condition_types_[i].c_str(), ex.what());
      return false;
    }
  }

  return true;
}

bool TriggerBroadcastNode::loadTriggerPlugin()
{
  auto node = shared_from_this();

  trigger_type_ = dc_util::get_str_type_param(node, trigger_id_, "plugin");
  trigger_if_all_conditions_ =
      dc_util::get_str_array_type_param(node, trigger_id_, "if_all_conditions", std::vector<std::string>());
  trigger_if_any_conditions_ =
      dc_util::get_str_array_type_param(node, trigger_id_, "if_any_conditions", std::vector<std::string>());
  trigger_if_none_conditions_ =
      dc_util::get_str_array_type_param(node, trigger_id_, "if_none_conditions", std::vector<std::string>());
  trigger_topic_ = dc_util::get_str_type_param(node, trigger_id_, "topic", std::string("/dc/flush"));
  trigger_polling_interval_ = dc_util::get_int_type_param(node, trigger_id_, "polling_interval", 100);

  try
  {
    RCLCPP_INFO_STREAM(get_logger(), "Creating trigger plugin "
                                          << trigger_id_ << ": Type " << trigger_type_ << ", Topic: " << trigger_topic_
                                          << ", Polling interval: " << trigger_polling_interval_);

    trigger_ = trigger_plugin_loader_.createUniqueInstance(trigger_type_);
    trigger_->configure(node, trigger_id_, conditions_, trigger_if_all_conditions_, trigger_if_any_conditions_,
                        trigger_if_none_conditions_);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(get_logger(),
                 "Failed to create trigger %s of type %s."
                 " Exception: %s",
                 trigger_id_.c_str(), trigger_type_.c_str(), ex.what());
    return false;
  }

  return true;
}

std::string TriggerBroadcastNode::mintIncidentId()
{
  uuid_t uuid_obj;
  char uuid_str[37];
  uuid_generate(uuid_obj);
  uuid_unparse(uuid_obj, uuid_str);
  return std::string(uuid_str);
}

void TriggerBroadcastNode::checkTrigger()
{
  if (!trigger_ || !trigger_->checkFired())
  {
    return;
  }

  dc_interfaces::msg::FlushEvent msg;
  msg.incident_id = mintIncidentId();
  msg.stamp = get_clock()->now();

  RCLCPP_INFO_STREAM(get_logger(), "Trigger '" << trigger_id_ << "' fired, incident_id=" << msg.incident_id);

  flush_pub_->publish(msg);
}

nav2_util::CallbackReturn TriggerBroadcastNode::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  auto node = shared_from_this();

  if (!loadConditionPlugins())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (!loadTriggerPlugin())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  flush_pub_ = node->create_publisher<dc_interfaces::msg::FlushEvent>(trigger_topic_, rclcpp::QoS(10));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TriggerBroadcastNode::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  auto node = shared_from_this();

  flush_pub_->on_activate();
  trigger_->activate();

  client_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  check_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(trigger_polling_interval_), [this] { checkTrigger(); }, client_cb_group_);

  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TriggerBroadcastNode::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  check_timer_.reset();
  trigger_->deactivate();
  flush_pub_->on_deactivate();

  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TriggerBroadcastNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  if (trigger_)
  {
    trigger_->cleanup();
    trigger_.reset();
  }
  for (auto& condition : conditions_)
  {
    condition.second->cleanup();
  }
  conditions_.clear();
  flush_pub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TriggerBroadcastNode::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace trigger_broadcast_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(trigger_broadcast_node::TriggerBroadcastNode)
