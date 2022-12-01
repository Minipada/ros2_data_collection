#ifndef DC_DESTINATIONS__DESTINATION_HPP_
#define DC_DESTINATIONS__DESTINATION_HPP_

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "dc_core/destination.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <nlohmann/json.hpp>

namespace dc_destinations
{

using namespace std::chrono_literals;  // NOLINT
using json = nlohmann::json;

/**
 * @class nav2_behaviors::Behavior
 * @brief An action server Behavior base class implementing the action server and basic factory.
 */
// template <typename ActionT>
class Destination : public dc_core::Destination
{
public:
  //   using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  /**
   * @brief A Destination constructor
   */
  Destination()
  {
  }

  virtual ~Destination() = default;

  // an opportunity for derived classes to do something on configuration
  // if they chose
  virtual void onConfigure()
  {
  }

  // an opportunity for derived classes to do something on configuration
  // if they chose
  virtual void onDestinationConfigure()
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

  virtual void sendData(dc_interfaces::msg::StringStamped::SharedPtr msg)
  {
    RCLCPP_DEBUG(logger_, "msg: %s", msg->data.c_str());
  }

  void dataCb(dc_interfaces::msg::StringStamped::SharedPtr msg)
  {
    sendData(msg);
  }

  // configure the server on lifecycle setup
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                 const std::vector<std::string>& inputs, const bool& debug) override
  {
    node_ = parent;
    auto node = node_.lock();

    inputs_ = inputs;
    debug_ = debug;

    logger_ = node->get_logger();

    for (auto& input : inputs_)
    {
      subscriptions_.push_back(node->create_subscription<dc_interfaces::msg::StringStamped>(
          input.c_str(), 10, std::bind(&Destination::dataCb, this, std::placeholders::_1)));
    }

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    destination_name_ = name;

    onDestinationConfigure();
    onConfigure();

    RCLCPP_INFO(logger_, "Done configuring %s", destination_name_.c_str());
  }

  // Cleanup server on lifecycle transition
  void cleanup() override
  {
    onCleanup();
  }

  // Activate server on lifecycle transition
  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating %s", destination_name_.c_str());

    enabled_ = true;
  }

  // Deactivate server on lifecycle transition
  void deactivate() override
  {
    enabled_ = false;
  }

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string destination_name_;
  std::vector<std::string> inputs_;
  double enabled_;

  json custom_params_;

  // Clock
  rclcpp::Clock steady_clock_{ RCL_STEADY_TIME };

  // Timer
  rclcpp::TimerBase::SharedPtr collect_timer_;
  std::vector<rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr> subscriptions_;

  // Logger
  rclcpp::Logger logger_{ rclcpp::get_logger("dc_destinations") };

  bool debug_{ false };

  // CB
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
};

}  // namespace dc_destinations

#endif  // DC_DESTINATIONS__DESTINATION_HPP_
