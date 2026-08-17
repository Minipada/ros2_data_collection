// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/driving_type.hpp"

namespace dc_measurements
{

namespace
{
// The documented, closed set of strings this Measurement ever emits as "mode" -- downstream
// grouping/dashboards key off these exact values, so a mapping targeting anything else is a
// configuration error rather than silently widening the set.
bool isValidMode(const std::string& mode)
{
  return mode == "autonomous" || mode == "manual" || mode == "teleop" || mode == "unknown";
}
}  // namespace

DrivingType::DrivingType() : dc_measurements::Measurement()
{
}

DrivingType::~DrivingType() = default;

void DrivingType::onConfigure()
{
  auto node = getNode();
  mode_topic_ = dc_util::get_str_type_param(node, measurement_name_, "mode_topic", "");
  value_mapping_from_ =
      dc_util::get_str_array_type_param(node, measurement_name_, "value_mapping_from", std::vector<std::string>{});
  value_mapping_to_ =
      dc_util::get_str_array_type_param(node, measurement_name_, "value_mapping_to", std::vector<std::string>{});
  velocity_topics_ =
      dc_util::get_str_array_type_param(node, measurement_name_, "velocity_topics", std::vector<std::string>{});
  velocity_modes_ =
      dc_util::get_str_array_type_param(node, measurement_name_, "velocity_modes", std::vector<std::string>{});
  velocity_timeout_s_ = dc_util::get_double_type_param(node, measurement_name_, "velocity_timeout_s", 1.0);

  // No mode has been observed yet: emit "unknown" rather than suppressing the Record, so
  // downstream consumers can tell "not yet known" apart from "no data collected at all".
  current_mode_ = "unknown";

  bool has_mode_topic = !mode_topic_.empty();
  bool has_velocity_sources = !velocity_topics_.empty();

  if (has_mode_topic && has_velocity_sources)
  {
    throw std::runtime_error{ "DrivingType: configure either 'mode_topic' or 'velocity_topics', not both" };
  }

  if (has_mode_topic)
  {
    if (value_mapping_from_.size() != value_mapping_to_.size())
    {
      RCLCPP_ERROR_STREAM(logger_, "DrivingType: 'value_mapping_from' and 'value_mapping_to' must be the same "
                                   "size, ignoring the mapping entirely");
    }
    else
    {
      for (size_t i = 0; i < value_mapping_from_.size(); ++i)
      {
        if (!isValidMode(value_mapping_to_[i]))
        {
          RCLCPP_ERROR_STREAM(logger_, "DrivingType: '"
                                           << value_mapping_to_[i]
                                           << "' is not one of the supported modes (autonomous, manual, teleop, "
                                              "unknown), ignoring mapping for raw value '"
                                           << value_mapping_from_[i] << "'");
          continue;
        }
        value_mapping_[value_mapping_from_[i]] = value_mapping_to_[i];
      }
    }

    mode_subscription_ = node->create_subscription<std_msgs::msg::String>(
        mode_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&DrivingType::modeTopicCb, this, std::placeholders::_1));
  }
  else if (has_velocity_sources)
  {
    if (velocity_topics_.size() != velocity_modes_.size())
    {
      throw std::runtime_error{ "DrivingType: 'velocity_topics' and 'velocity_modes' must be the same size" };
    }

    for (size_t i = 0; i < velocity_topics_.size(); ++i)
    {
      if (!isValidMode(velocity_modes_[i]))
      {
        RCLCPP_ERROR_STREAM(logger_, "DrivingType: '"
                                         << velocity_modes_[i]
                                         << "' is not one of the supported modes (autonomous, manual, teleop, "
                                            "unknown), ignoring velocity source '"
                                         << velocity_topics_[i] << "'");
        continue;
      }

      std::string mode = velocity_modes_[i];
      velocity_subscriptions_.push_back(node->create_subscription<geometry_msgs::msg::Twist>(
          velocity_topics_[i], rclcpp::SystemDefaultsQoS(),
          [this, mode](const geometry_msgs::msg::Twist&) { velocitySourceCb(mode); }));
    }
  }
}

void DrivingType::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "driving_type.json");
  }
}

void DrivingType::modeTopicCb(const std_msgs::msg::String& msg)
{
  auto it = value_mapping_.find(msg.data);
  if (it == value_mapping_.end())
  {
    RCLCPP_DEBUG_STREAM(logger_, "DrivingType: no mapping for raw mode value '"
                                     << msg.data << "', keeping current mode '" << current_mode_ << "'");
    return;
  }

  current_mode_ = it->second;
}

void DrivingType::velocitySourceCb(const std::string& mode)
{
  current_mode_ = mode;
  velocity_observed_ = true;
  last_velocity_time_ns_ = getNode()->get_clock()->now().nanoseconds();
}

dc_interfaces::msg::StringStamped DrivingType::collect()
{
  if (!velocity_subscriptions_.empty())
  {
    // No velocity source has published recently enough to be considered "currently driving"
    // through it: fall back to unknown rather than keep reporting a stale mode forever.
    bool stale = true;
    if (velocity_observed_)
    {
      int64_t now_ns = getNode()->get_clock()->now().nanoseconds();
      double elapsed_s = static_cast<double>(now_ns - last_velocity_time_ns_) / 1e9;
      stale = elapsed_s > velocity_timeout_s_;
    }

    if (stale)
    {
      current_mode_ = "unknown";
    }
  }

  json data_json;
  data_json["mode"] = current_mode_;

  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.group_key = group_key_;
  msg.data = data_json.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::DrivingType, dc_core::Measurement)
