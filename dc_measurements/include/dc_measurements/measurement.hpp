#ifndef DC_MEASUREMENTS__MEASUREMENT_HPP_
#define DC_MEASUREMENTS__MEASUREMENT_HPP_

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <utility>

#include "dc_core/measurement.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace dc_measurements
{

enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

using namespace std::chrono_literals;  // NOLINT
using json = nlohmann::json;

/**
 * @class nav2_behaviors::Behavior
 * @brief An action server Behavior base class implementing the action server and basic factory.
 */
// template <typename ActionT>
class Measurement : public dc_core::Measurement
{
public:
  //   using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  /**
   * @brief A Measurement constructor
   */
  Measurement()
  {
  }

  virtual ~Measurement() = default;

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

  void collectAndPublish()
  {
    if (enabled_)
    {
      auto msg = collect();
      RCLCPP_DEBUG(logger_, "Measurement: %s, msg: %s", measurement_name_.c_str(),
                   dc_interfaces::msg::to_yaml(msg).c_str());
      data_pub_->publish(msg);
    }
  }

  virtual dc_interfaces::msg::StringStamped collect() = 0;

  // configure the server on lifecycle setup
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                 std::shared_ptr<tf2_ros::Buffer> tf, const std::string& measurement_plugin,
                 const std::string& topic_output, const int& polling_interval, const bool& debug,
                 const rclcpp::CallbackGroup::SharedPtr& timer_cb_group) override
  {
    node_ = parent;
    auto node = node_.lock();

    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    measurement_plugin_ = measurement_plugin;
    tf_ = tf;
    measurement_name_ = name;
    topic_output_ = topic_output;
    polling_interval_ = polling_interval;
    debug_ = debug;
    timer_cb_group_ = timer_cb_group;

    data_pub_ = node->create_publisher<dc_interfaces::msg::StringStamped>(
        std::string("/dc/measurement/") + measurement_name_, 1);
    collect_timer_ =
        node->create_wall_timer(std::chrono::milliseconds(polling_interval_),
                                std::bind(&dc_measurements::Measurement::collectAndPublish, this), timer_cb_group_);

    RCLCPP_INFO(logger_, "Done configuring %s", measurement_name_.c_str());

    onConfigure();
  }

  // Cleanup server on lifecycle transition
  void cleanup() override
  {
    data_pub_.reset();
    onCleanup();
  }

  // Activate server on lifecycle transition
  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating %s", measurement_name_.c_str());

    data_pub_->on_activate();
    enabled_ = true;
  }

  // Deactivate server on lifecycle transition
  void deactivate() override
  {
    data_pub_->on_deactivate();
    enabled_ = false;
  }

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string measurement_name_;
  std::string measurement_plugin_;
  rclcpp_lifecycle::LifecyclePublisher<dc_interfaces::msg::StringStamped>::SharedPtr data_pub_;
  double enabled_;
  int polling_interval_;
  std::string topic_output_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  bool debug_;

  // Timer
  rclcpp::TimerBase::SharedPtr collect_timer_;

  // Logger
  rclcpp::Logger logger_{ rclcpp::get_logger("dc_measurements") };

  // CB
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__MEASUREMENT_HPP_
