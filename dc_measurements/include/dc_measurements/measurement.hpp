// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__MEASUREMENT_HPP_
#define DC_MEASUREMENTS__MEASUREMENT_HPP_

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "dc_core/condition.hpp"
#include "dc_core/measurement.hpp"
#include "dc_interfaces/msg/condition.hpp"
#include "dc_interfaces/msg/flush_event.hpp"
#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using namespace std::chrono_literals;  // NOLINT

namespace dc_measurements
{

/**
 * @class dc_measurements::Measurement
 * @brief The ROS driver of a Measurement plugin: the thin half of the framework split (#499).
 *
 * Everything decidable without ROS lives in MeasurementCore (one compiled copy shared by every
 * plugin, directly testable). This class owns the ROS wiring -- publisher, polling and release
 * timers, flush subscription, lifecycle transitions, node parameters -- and the plugin-facing
 * surface: plugins keep inheriting exactly this one class and overriding collect() plus the
 * onConfigure/onCleanup/onFailedValidation hooks.
 */
class Measurement : public dc_core::Measurement
{
public:
  Measurement()
  {
  }

  ~Measurement() override = default;

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

  // an opportunity for derived classes to do something on failed validation
  // if they chose
  virtual void onFailedValidation(json data_json)
  {
    (void)data_json;  // Ignore error of variable being unused
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

  std::string getSavePath(const std::string& path_parameter)
  {
    auto rclcpp_time = rclcpp::Clock().now();
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> t{ std::chrono::nanoseconds(
        rclcpp_time.nanoseconds()) };
    std::time_t newt = std::chrono::system_clock::to_time_t(t);
    std::stringstream trans_time;

    auto node = getNode();
    trans_time << std::put_time(localtime(&newt),
                                node->get_parameter(measurement_name_ + "." + path_parameter).as_string().c_str());
    std::string fmt_time = trans_time.str();

    return fmt_time;
  }

  std::string getSavePath(const std::string& path_parameter, const rclcpp::Time& now)
  {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> t{ std::chrono::nanoseconds(
        now.nanoseconds()) };
    std::time_t newt = std::chrono::system_clock::to_time_t(t);
    std::stringstream trans_time;

    auto node = getNode();
    trans_time << std::put_time(localtime(&newt),
                                node->get_parameter(measurement_name_ + "." + path_parameter).as_string().c_str());
    std::string fmt_time = trans_time.str();

    return fmt_time;
  }

  void validateSchema(const std::string& package_name, const std::string& json_filename)
  {
    core_.validateSchema(package_name, json_filename);
  }

  void validateSchema(const std::string& json_schema_path)
  {
    core_.validateSchema(json_schema_path);
  }

  static std::string snakeCase(const std::string& camel)
  {
    return MeasurementCore::snakeCase(camel);
  }

  static std::string defaultSchemaFile(const std::string& plugin_lookup_name)
  {
    return MeasurementCore::defaultSchemaFile(plugin_lookup_name);
  }

  void publish(dc_interfaces::msg::StringStamped msg)
  {
    auto msg_copy = msg;
    core_.publish(msg.data, msg.group_key, rclcpp::Time(msg.header.stamp).nanoseconds(), gateLookup(msg_copy),
                  conditionLookup(msg_copy), std::chrono::system_clock::now());
    if (core_.collectionFinished())
    {
      collect_timer_.reset();
    }
  }

  void publishFromMsg(const dc_interfaces::msg::StringStamped& msg)
  {
    if (core_.collectible())
    {
      publish(msg);
    }
  }

  void collectAndPublish()
  {
    dc_interfaces::msg::StringStamped msg = collect();
    if (core_.collectible())
    {
      if (core_.hasReleaser())
      {
        core_.offerSample(msg.data, std::chrono::system_clock::now());
      }
      else
      {
        publish(msg);
      }
    }
  }

  virtual dc_interfaces::msg::StringStamped collect() = 0;

  // configure the server on lifecycle setup
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                 const std::map<std::string, std::shared_ptr<dc_core::Condition>>& conditions,
                 std::shared_ptr<tf2_ros::Buffer> tf, const dc_core::MeasurementConfig& config) override
  {
    node_ = parent;
    auto node = node_.lock();

    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    measurement_plugin_ = config.measurement_plugin;
    conditions_ = conditions;
    tf_ = tf;
    measurement_name_ = name;
    topic_output_ = config.topic_output;
    polling_interval_ = config.polling_interval;
    debug_ = config.debug;
    enable_validator_ = config.enable_validator;
    group_key_ = config.group_key;
    init_collect_ = config.init_collect;
    remote_keys_ = config.remote_keys;
    remote_prefixes_ = config.remote_prefixes;
    all_base_path_ = config.all_base_path;
    all_base_path_expanded_ = config.all_base_path_expanded;
    save_local_base_path_ = config.save_local_base_path;
    save_local_base_path_expanded_ = config.save_local_base_path_expanded;

    // The pipeline runs ROS-free in MeasurementCore; this wiring builds its inputs and outputs.
    MeasurementCore::Config core_config;
    core_config.measurement_name = measurement_name_;
    core_config.measurement_plugin = measurement_plugin_;
    core_config.group_key = group_key_;
    core_config.enable_validator = enable_validator_;
    core_config.json_schema_path = config.json_schema_path;
    core_config.save_local_base_path_expanded = save_local_base_path_expanded_;
    core_config.if_all_conditions = config.if_all_conditions;
    core_config.if_any_conditions = config.if_any_conditions;
    core_config.if_none_conditions = config.if_none_conditions;
    core_config.gate_condition = config.gate_condition;
    core_config.init_max_measurements = config.init_max_measurements;
    core_config.condition_max_measurements = config.condition_max_measurements;
    core_config.nested = config.nested;
    core_config.flatten = config.flatten;
    core_config.run_id_enabled = config.run_id_enabled;
    core_config.run_id = config.run_id;
    core_config.include_measurement_name = config.include_measurement_name;
    core_config.include_measurement_plugin = config.include_measurement_plugin;
    core_config.tags = config.tags;
    core_config.custom_keys = config.custom_keys;
    core_config.buffer_duration_sec = config.buffer_duration_sec;
    core_config.post_roll_duration_sec = config.post_roll_duration_sec;
    core_config.cooldown_sec = config.cooldown_sec;
    core_config.max_flush_rate_hz = config.max_flush_rate_hz;
    core_.configure(
        core_config, [this](const RecordOut& out) { publishOut(out); },
        [this](const json& data_json) { onFailedValidation(data_json); },
        [this](LogLevel level, const std::string& message) { log(level, message); });

    if (topic_output_.empty())
    {
      topic_output_ = std::string("/dc/measurement/") + measurement_name_;
    }

    data_pub_ = node->create_publisher<dc_interfaces::msg::StringStamped>(topic_output_, 1);
    client_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    collect_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(polling_interval_), [this] { collectAndPublish(); }, client_cb_group_);

    if (core_.hasReleaser())
    {
      flush_sub_ = node->create_subscription<dc_interfaces::msg::FlushEvent>(
          config.flush_topic.empty() ? std::string("/dc/flush") : config.flush_topic, 10,
          [this](const dc_interfaces::msg::FlushEvent& event) {
            core_.onFlushEvent(event.incident_id, std::chrono::system_clock::now());
          });
      if (config.max_flush_rate_hz > 0.0)
      {
        // A rate-limited release is drained by tick(), and the polling timer alone would drain it
        // at the polling rate -- in bursts, and never faster than collection however high
        // max_flush_rate_hz is. This timer paces the release properly (#289).
        release_timer_ = node->create_wall_timer(
            durationFromSeconds(1.0 / config.max_flush_rate_hz),
            [this] { core_.tickRelease(std::chrono::system_clock::now()); }, client_cb_group_);
      }
    }

    RCLCPP_INFO(logger_, "Done configuring %s", measurement_name_.c_str());

    if (enable_validator_)
    {
      if (config.json_schema_path.empty())
      {
        core_.validateDefaultSchema();
      }
      else
      {
        core_.validateSchema(config.json_schema_path);
      }
    }
    // If error during measurement initialization, stop it from publishing
    try
    {
      onConfigure();
    }
    catch (std::runtime_error& error)
    {
      core_.markConfigureFailed();
      collect_timer_.reset();
    }

    if (enable_validator_ && core_.schemaEmpty())
    {
      throw std::runtime_error{ "Enabled validation but didn't configure schema!" };
    }
  }

  // Cleanup server on lifecycle transition
  void cleanup() override
  {
    data_pub_.reset();
    flush_sub_.reset();
    // Before the core's releaser it ticks, so the timer can't fire on a null one.
    release_timer_.reset();
    core_.teardown();
    onCleanup();
  }

  // Activate server on lifecycle transition
  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating measurement %s", measurement_name_.c_str());

    data_pub_->on_activate();
    // Re-enable collection: deactivate() cleared it, and a deactivate -> activate cycle resumes
    // where the old never-reset flag silenced the Measurement permanently (#499). A Measurement
    // whose onConfigure() hook failed stays silent regardless -- see MeasurementCore::collectible().
    core_.setEnabled(true);

    if (core_.collectible() && init_collect_)
    {
      collectAndPublish();
    }
  }

  // Deactivate server on lifecycle transition
  void deactivate() override
  {
    data_pub_->on_deactivate();
    core_.setEnabled(false);
  }

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string measurement_name_;
  std::string measurement_plugin_;
  std::string group_key_;
  bool debug_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  // Publish data
  int polling_interval_;
  rclcpp_lifecycle::LifecyclePublisher<dc_interfaces::msg::StringStamped>::SharedPtr data_pub_;
  rclcpp::TimerBase::SharedPtr collect_timer_;
  std::string topic_output_;

  // Parameters
  bool init_collect_;
  std::vector<std::string> remote_keys_;
  std::vector<std::string> remote_prefixes_;
  std::string all_base_path_;
  std::string all_base_path_expanded_;
  std::string save_local_base_path_;
  std::string save_local_base_path_expanded_;

  // Conditions, resolved per collection into the lookups the core's gate and ConditionSet consume
  std::map<std::string, std::shared_ptr<dc_core::Condition>> conditions_;

  // Logger
  rclcpp::Logger logger_{ rclcpp::get_logger("dc_measurements") };

  // Validation
  bool enable_validator_;

  // The pipeline, ROS-free and shared by every plugin (#499)
  MeasurementCore core_;

private:
  // State of one configured Condition, as dc_core::ConditionSet asks for it. A name that is not a
  // configured Condition reads as false rather than dereferencing a null plugin, which is what
  // the previous conditions_[name]->getState(msg) did on a typo'd name.
  dc_core::ConditionStateLookup conditionLookup(const dc_interfaces::msg::StringStamped& msg) const
  {
    return [this, &msg](const std::string& condition_name) {
      const auto condition_it = conditions_.find(condition_name);
      if (condition_it == conditions_.end() || !condition_it->second)
      {
        RCLCPP_ERROR_STREAM(logger_, "Measurement " << measurement_name_ << ": '" << condition_name
                                                    << "' is not a configured condition; treating it as false.");
        return false;
      }
      return condition_it->second->getState(msg);
    };
  }

  // Same lookup, worded for the gate: a gate_condition naming nothing holds all collection back.
  dc_core::ConditionStateLookup gateLookup(const dc_interfaces::msg::StringStamped& msg) const
  {
    return [this, &msg](const std::string& condition_name) {
      const auto condition_it = conditions_.find(condition_name);
      if (condition_it == conditions_.end() || !condition_it->second)
      {
        RCLCPP_ERROR_STREAM(logger_, "Measurement " << measurement_name_ << ": gate_condition '" << condition_name
                                                    << "' is not a configured condition; holding all collection back.");
        return false;
      }
      return condition_it->second->getState(msg);
    };
  }

  // PublishFn target: every Record the pipeline emits -- live or released from the incident
  // buffer -- goes out through the one lifecycle publisher.
  void publishOut(const RecordOut& out)
  {
    dc_interfaces::msg::StringStamped msg;
    msg.data = out.data;
    msg.group_key = out.group_key;
    msg.incident_id = out.incident_id;
    msg.header.stamp = rclcpp::Time(out.stamp_ns);
    data_pub_->publish(msg);
  }

  void log(LogLevel level, const std::string& message)
  {
    switch (level)
    {
      case LogLevel::Debug:
        RCLCPP_DEBUG(logger_, "%s", message.c_str());
        break;
      case LogLevel::Info:
        RCLCPP_INFO(logger_, "%s", message.c_str());
        break;
      case LogLevel::Warn:
        RCLCPP_WARN(logger_, "%s", message.c_str());
        break;
      case LogLevel::Error:
        RCLCPP_ERROR(logger_, "%s", message.c_str());
        break;
    }
  }

  rclcpp::Subscription<dc_interfaces::msg::FlushEvent>::SharedPtr flush_sub_;
  // Paces a rate-limited release; only created when max_flush_rate_hz > 0 (#289).
  rclcpp::TimerBase::SharedPtr release_timer_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__MEASUREMENT_HPP_
