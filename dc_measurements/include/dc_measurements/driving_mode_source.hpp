// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__DRIVING_MODE_SOURCE_HPP_
#define DC_MEASUREMENTS__DRIVING_MODE_SOURCE_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "dc_measurements/source_adapter.hpp"
#include "dc_util/node_utils.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace dc_measurements
{

/**
 * @brief The documented, closed set of driving modes DC ever reports. Downstream
 * grouping/dashboards key off these exact values, so a mapping targeting anything else is a
 * configuration error rather than a silently widened set.
 */
inline bool isValidDrivingMode(const std::string& mode)
{
  return mode == "autonomous" || mode == "manual" || mode == "teleop" || mode == "unknown";
}

/**
 * @class dc_measurements::DrivingModeSource
 * @brief The one way DC turns robot topics into a driving mode, shared by every Measurement that
 * needs one.
 *
 * `driving_type` reports the mode and `intervention` (#362) projects its transitions, so they must
 * never disagree about when the robot was autonomous: both drive this, rather than each
 * subscribing on its own. Supports the two shapes documented for `driving_type` -- a dedicated
 * mode topic mapped through `value_mapping_from`/`value_mapping_to`, or inference from whichever
 * of `velocity_topics` last published -- chosen by which parameters are set; configuring both is a
 * configuration error.
 */
class DrivingModeSource
{
public:
  /**
   * @brief Read the mode-source parameters of `measurement_name` and subscribe accordingly.
   * @throws std::runtime_error when both shapes are configured, or when the velocity lists
   * disagree in size.
   */
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& measurement_name,
                 const rclcpp::Logger& logger)
  {
    clock_ = node->get_clock();
    mode_topic_ = dc_util::get_str_type_param(node, measurement_name, "mode_topic", "");
    value_mapping_from_ =
        dc_util::get_str_array_type_param(node, measurement_name, "value_mapping_from", std::vector<std::string>{});
    value_mapping_to_ =
        dc_util::get_str_array_type_param(node, measurement_name, "value_mapping_to", std::vector<std::string>{});
    velocity_topics_ =
        dc_util::get_str_array_type_param(node, measurement_name, "velocity_topics", std::vector<std::string>{});
    velocity_modes_ =
        dc_util::get_str_array_type_param(node, measurement_name, "velocity_modes", std::vector<std::string>{});
    velocity_timeout_s_ = dc_util::get_double_type_param(node, measurement_name, "velocity_timeout_s", 1.0);

    const bool has_mode_topic = !mode_topic_.empty();
    const bool has_velocity_sources = !velocity_topics_.empty();

    if (has_mode_topic && has_velocity_sources)
    {
      throw std::runtime_error{ measurement_name + ": configure either 'mode_topic' or 'velocity_topics', not both" };
    }

    if (has_mode_topic)
    {
      configureModeTopic(node, measurement_name, logger);
    }
    else if (has_velocity_sources)
    {
      configureVelocitySources(node, measurement_name, logger);
    }
  }

  /**
   * @brief The mode as of now.
   *
   * "unknown" until a mode has been observed, and again once every velocity source has gone
   * quiet for longer than `velocity_timeout_s` -- a stale source must not keep reporting a mode
   * the robot left.
   */
  std::string mode()
  {
    const auto latest = mode_observations_.latest();
    if (!latest)
    {
      return "unknown";
    }
    if (!velocity_subscriptions_.empty())
    {
      const int64_t now_ns = clock_->now().nanoseconds();
      if (static_cast<double>(now_ns - latest->second) / 1e9 > velocity_timeout_s_)
      {
        return "unknown";
      }
    }
    return latest->first;
  }

private:
  void configureModeTopic(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& measurement_name,
                          const rclcpp::Logger& logger)
  {
    if (value_mapping_from_.size() != value_mapping_to_.size())
    {
      RCLCPP_ERROR_STREAM(logger, measurement_name
                                      << ": 'value_mapping_from' and 'value_mapping_to' must be the same size, "
                                         "ignoring the mapping entirely");
    }
    else
    {
      for (size_t i = 0; i < value_mapping_from_.size(); ++i)
      {
        if (!isValidDrivingMode(value_mapping_to_[i]))
        {
          RCLCPP_ERROR_STREAM(logger, measurement_name
                                          << ": '" << value_mapping_to_[i]
                                          << "' is not one of the supported modes (autonomous, manual, teleop, "
                                             "unknown), ignoring mapping for raw value '"
                                          << value_mapping_from_[i] << "'");
          continue;
        }
        value_mapping_[value_mapping_from_[i]] = value_mapping_to_[i];
      }
    }

    mode_subscription_ = node->create_subscription<std_msgs::msg::String>(
        mode_topic_, rclcpp::SystemDefaultsQoS(), [this, logger, measurement_name](const std_msgs::msg::String& msg) {
          const auto it = value_mapping_.find(msg.data);
          if (it == value_mapping_.end())
          {
            RCLCPP_DEBUG_STREAM(logger, measurement_name << ": no mapping for raw mode value '" << msg.data
                                                         << "', keeping the last observed mode");
            return;
          }
          mode_observations_.push({ it->second, clock_->now().nanoseconds() });
        });
  }

  void configureVelocitySources(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                const std::string& measurement_name, const rclcpp::Logger& logger)
  {
    if (velocity_topics_.size() != velocity_modes_.size())
    {
      throw std::runtime_error{ measurement_name + ": 'velocity_topics' and 'velocity_modes' must be the same size" };
    }

    for (size_t i = 0; i < velocity_topics_.size(); ++i)
    {
      if (!isValidDrivingMode(velocity_modes_[i]))
      {
        RCLCPP_ERROR_STREAM(logger, measurement_name
                                        << ": '" << velocity_modes_[i]
                                        << "' is not one of the supported modes (autonomous, manual, teleop, "
                                           "unknown), ignoring velocity source '"
                                        << velocity_topics_[i] << "'");
        continue;
      }

      const std::string mode = velocity_modes_[i];
      velocity_subscriptions_.push_back(node->create_subscription<geometry_msgs::msg::Twist>(
          velocity_topics_[i], rclcpp::SystemDefaultsQoS(), [this, mode](const geometry_msgs::msg::Twist&) {
            mode_observations_.push({ mode, clock_->now().nanoseconds() });
          }));
    }
  }

  // The clock, not the node: the node owns the Measurement that owns this, so holding the node
  // back would be a reference cycle.
  rclcpp::Clock::SharedPtr clock_;

  // Shape 1: a dedicated topic carrying a raw mode value, mapped through value_mapping_.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
  std::string mode_topic_;
  std::vector<std::string> value_mapping_from_;
  std::vector<std::string> value_mapping_to_;
  std::map<std::string, std::string> value_mapping_;

  // Shape 2: infer the mode from whichever velocity source last published.
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> velocity_subscriptions_;
  std::vector<std::string> velocity_topics_;
  std::vector<std::string> velocity_modes_;
  double velocity_timeout_s_{ 1.0 };

  // The one piece of shared state: which mode was observed last, and when. The subscription
  // callbacks write through the adapter, the polling timer reads in mode() -- different callback
  // groups under a multi-threaded executor, so the handoff is the adapter's (#502). Nothing
  // observed yet reads back "unknown", so a consumer can tell "not yet known" apart from "no
  // data collected at all".
  SourceAdapter<std::pair<std::string, int64_t>> mode_observations_{ 1 };
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__DRIVING_MODE_SOURCE_HPP_
