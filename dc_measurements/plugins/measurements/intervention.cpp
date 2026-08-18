// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/intervention.hpp"

namespace dc_measurements
{

namespace
{

constexpr const char* kAutonomousMode = "autonomous";

// The fixed set `dc_kpi_intervention_events` (#369) already matches literally -- not a
// configurable list, because a custom human mode the SQL doesn't know about would silently never
// count as a takeover downstream.
bool isHumanMode(const std::string& mode)
{
  return mode == "manual" || mode == "teleop";
}

double secondsOf(const std::chrono::system_clock::duration& d)
{
  return std::chrono::duration<double>(d).count();
}

}  // namespace

Intervention::Intervention() : dc_measurements::Measurement()
{
}

Intervention::~Intervention() = default;

void Intervention::onConfigure()
{
  mode_source_.configure(getNode(), measurement_name_, logger_);
}

void Intervention::onCleanup()
{
  const auto open = detector_.openInterval(std::chrono::system_clock::now());
  if (open.has_value() && isHumanMode(open->state))
  {
    RCLCPP_WARN_STREAM(logger_, measurement_name_ << ": collection stopped during a takeover ("
                                                  << secondsOf(open->elapsed)
                                                  << "s so far); no closing Record will be published");
  }
}

void Intervention::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "intervention.json");
  }
}

dc_interfaces::msg::StringStamped Intervention::collect()
{
  auto node = getNode();
  const rclcpp::Time stamp = node->get_clock()->now();
  const std::chrono::system_clock::time_point now{ std::chrono::nanoseconds(stamp.nanoseconds()) };

  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = stamp;
  msg.group_key = group_key_;

  // Empty data on every poll that saw no takeover boundary: this Measurement reports events, not
  // a level, and publish() already treats an empty Record as "nothing to report" (#279).
  const auto transition = detector_.update(mode_source_.mode(), now);
  if (!transition.has_value())
  {
    return msg;
  }

  const bool starts = transition->from == kAutonomousMode && isHumanMode(transition->to);
  const bool ends = isHumanMode(transition->from) && transition->to == kAutonomousMode;
  if (!starts && !ends)
  {
    // Autonomy handing over to nothing (a mode signal gone stale to "unknown"), or one human mode
    // replacing another directly: a real transition, but not a takeover boundary.
    return msg;
  }

  json data_json;
  data_json["event"] = starts ? "start" : "end";
  data_json["from_mode"] = transition->from;
  data_json["to_mode"] = transition->to;
  // The dwell of `from_mode`: on a start Record, how long the robot had been autonomous before
  // the takeover; on an end Record, how long the takeover itself lasted -- the same field means
  // both, because `from_mode` is exactly the state that just ended either way (#369's
  // dc_kpi_intervention_events reads it this way, gated by is_start/is_end).
  data_json["previous_duration"] = secondsOf(transition->dwell);
  data_json["sequence"] = transition->sequence;
  data_json["open"] = starts;

  msg.data = data_json.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Intervention, dc_core::Measurement)
