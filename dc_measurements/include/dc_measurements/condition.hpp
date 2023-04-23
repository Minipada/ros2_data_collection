#ifndef DC_MEASUREMENTS__CONDITION_HPP_
#define DC_MEASUREMENTS__CONDITION_HPP_

#include <yaml-cpp/yaml.h>

#include <nlohmann/json.hpp>

#include "dc_core/condition.hpp"
#include "dc_interfaces/msg/condition.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_conditions
{

enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

using json = nlohmann::json;

using namespace std::chrono_literals;  // NOLINT

/**
 * @class nav2_behaviors::Behavior
 * @brief An action server Behavior base class implementing the action server and basic factory.
 */
// template <typename ActionT>
class Condition : public dc_core::Condition
{
public:
  /**
   * @brief A Condition constructor
   */
  Condition()
  {
  }

  virtual ~Condition() = default;

  // an opportunity for derived classes to do something on configuration
  // if they chose
  virtual void onConfigure()
  {
  }

  // an opportunity for derived classes to do something on cleanup
  // if they chose
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

  // configure the server on lifecycle setup
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name) override
  {
    node_ = parent;
    auto node = node_.lock();

    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    condition_name_ = name;

    condition_pub_ =
        node->create_publisher<dc_interfaces::msg::Condition>(std::string("/dc/condition/") + condition_name_, 1);

    RCLCPP_INFO(logger_, "Done configuring %s", condition_name_.c_str());

    onConfigure();
  }

  // Cleanup server on lifecycle transition
  void cleanup() override
  {
    condition_pub_.reset();
    onCleanup();
  }

  // Activate server on lifecycle transition
  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating %s", condition_name_.c_str());

    enabled_ = true;
  }

  // Deactivate server on lifecycle transition
  void deactivate() override
  {
    enabled_ = false;
  }

  virtual bool getState(dc_interfaces::msg::StringStamped msg) override
  {
    (void)msg;  // Ignore error of variable being unused
    publishActive();
    return active_;
  }

  void publishActive() override
  {
    auto msg_condition = dc_interfaces::msg::Condition();
    msg_condition.data = active_;
    condition_pub_->publish(msg_condition);
  }
  bool active_{ false };

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string condition_name_;
  rclcpp::Publisher<dc_interfaces::msg::Condition>::SharedPtr condition_pub_;

  bool enabled_;
  std::string group_key_;
  std::vector<std::string> if_all_conditions_;
  std::vector<std::string> if_any_conditions_;
  std::vector<std::string> if_none_conditions_;
  bool init_collect_;
  bool enable_validator_;
  int init_max_measurements_;
  int polling_interval_;
  std::string topic_output_;
  int condition_max_measurements_;

  // Clock
  rclcpp::Clock steady_clock_{ RCL_STEADY_TIME };

  // Logger
  rclcpp::Logger logger_{ rclcpp::get_logger("dc_conditions") };
};

}  // namespace dc_conditions

#endif  // DC_MEASUREMENTS__CONDITION_HPP_
