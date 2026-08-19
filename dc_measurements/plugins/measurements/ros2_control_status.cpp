// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/ros2_control_status.hpp"

#include "lifecycle_msgs/msg/state.hpp"

namespace dc_measurements
{

namespace
{
constexpr size_t kMaxPendingRecords = 64;
constexpr const char* kControllerType = "controller";
constexpr const char* kHardwareComponentType = "hardware_component";

double secondsOf(const std::chrono::system_clock::duration& d)
{
  return std::chrono::duration<double>(d).count();
}

// The controller manager's own labels are not reproduced on every sample -- only the current
// entry's -- so a transition's `from` state is looked up from the id the detector held, the same
// way Fault turns a DiagnosticStatus level back into a name.
std::string stateLabel(uint8_t id)
{
  switch (id)
  {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      return "unconfigured";
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      return "inactive";
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      return "active";
    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      return "finalized";
    case lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING:
      return "configuring";
    case lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP:
      return "cleaningup";
    case lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN:
      return "shuttingdown";
    case lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING:
      return "activating";
    case lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING:
      return "deactivating";
    case lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING:
      return "errorprocessing";
    default:
      return "unknown";
  }
}

}  // namespace

Ros2ControlStatus::Ros2ControlStatus() : dc_measurements::Measurement()
{
}

Ros2ControlStatus::~Ros2ControlStatus() = default;

void Ros2ControlStatus::onConfigure()
{
  auto node = getNode();
  topic_ = dc_util::get_str_type_param(node, measurement_name_, "topic", "/controller_manager/activity");

  // Matches the controller manager's own `~/activity` publisher exactly: transient-local so a
  // late-joining subscriber still gets the current snapshot instead of only future changes.
  subscription_ = node->create_subscription<controller_manager_msgs::msg::ControllerManagerActivity>(
      topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&Ros2ControlStatus::activityCb, this, std::placeholders::_1));
}

void Ros2ControlStatus::onCleanup()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& [name, detector] : controller_detectors_)
  {
    if (detector.state().has_value() && *detector.state() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_WARN_STREAM(logger_, measurement_name_ << ": collection stopped with controller '" << name
                                                    << "' still active; no closing Record will be published");
    }
  }
  for (const auto& [name, detector] : hardware_component_detectors_)
  {
    if (detector.state().has_value() && *detector.state() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_WARN_STREAM(logger_, measurement_name_ << ": collection stopped with hardware component '" << name
                                                    << "' still active; no closing Record will be published");
    }
  }
}

void Ros2ControlStatus::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "ros2_control_status.json");
  }
}

void Ros2ControlStatus::processEntries(const std::vector<controller_manager_msgs::msg::NamedLifecycleState>& entries,
                                       const std::string& component_type, const rclcpp::Time& stamp,
                                       std::map<std::string, dc_common::StateTransitionDetector<uint8_t>>& detectors)
{
  const std::chrono::system_clock::time_point now{ std::chrono::nanoseconds(stamp.nanoseconds()) };

  for (const auto& entry : entries)
  {
    const auto transition = detectors[entry.name].update(entry.state.id, now);
    if (!transition.has_value())
    {
      continue;
    }

    const bool starts = transition->to == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    const bool ends = transition->from == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    if (!starts && !ends)
    {
      // Neither entering nor leaving `active` (e.g. unconfigured to inactive during startup): a
      // real transition, but not the boundary this Measurement reports.
      continue;
    }

    json data;
    data["seq"] = ++record_seq_;
    data["component_type"] = component_type;
    data["component"] = entry.name;
    data["event"] = starts ? "start" : "end";
    data["from_state"] = stateLabel(transition->from);
    data["to_state"] = stateLabel(transition->to);
    // The dwell of `from_state`: on a start Record, how long the component had been out of
    // `active` before this; on an end Record, how long it had been `active` -- the same field
    // means both, matching Intervention's `previous_duration` (#369's reasoning applies here too).
    data["previous_state_duration_s"] = secondsOf(transition->dwell);
    data["open"] = starts;

    pending_records_.emplace_back(std::move(data), stamp);
  }
}

void Ros2ControlStatus::activityCb(const controller_manager_msgs::msg::ControllerManagerActivity& msg)
{
  const rclcpp::Time stamp = getNode()->get_clock()->now();

  const std::lock_guard<std::mutex> lock(mutex_);
  processEntries(msg.controllers, kControllerType, stamp, controller_detectors_);
  processEntries(msg.hardware_components, kHardwareComponentType, stamp, hardware_component_detectors_);

  // One Record leaves per poll, so a controller manager flapping far faster than the polling
  // interval would otherwise queue without bound. The oldest goes first: the recent transitions
  // are the ones still worth reporting.
  while (pending_records_.size() > kMaxPendingRecords)
  {
    pending_records_.pop_front();
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                "Measurement "
                                    << measurement_name_
                                    << ": ros2_control_status Records are arriving faster than the polling interval "
                                       "can report them; dropping the oldest.");
  }
}

dc_interfaces::msg::StringStamped Ros2ControlStatus::collect()
{
  dc_interfaces::msg::StringStamped msg;
  msg.group_key = group_key_;

  const std::lock_guard<std::mutex> lock(mutex_);
  if (pending_records_.empty())
  {
    return msg;
  }

  auto record = std::move(pending_records_.front());
  pending_records_.pop_front();
  msg.header.stamp = record.second;
  msg.data = record.first.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Ros2ControlStatus, dc_core::Measurement)
