// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__ROS2_CONTROL_STATUS_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__ROS2_CONTROL_STATUS_HPP_

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "controller_manager_msgs/msg/named_lifecycle_state.hpp"
#include "dc_common/state_transition_detector.hpp"
#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/pending_record_queue.hpp"
#include "dc_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_measurements
{

/**
 * @class dc_measurements::Ros2ControlStatus
 * @brief One Record per controller/hardware component crossing into or out of `active`, derived
 * from the controller manager's `~/activity` topic (transient-local, republished on every
 * lifecycle change) rather than polling `list_controllers`/`list_hardware_components`. Built on
 * the same dc_common::StateTransitionDetector as Intervention and Fault (#362, #365): one
 * detector per controller/hardware component, keyed the way Fault keys its per-component map, so
 * they fault and recover independently; the `start`/`end` boundary it watches for is the same
 * shape Intervention already reports (#395).
 */
class Ros2ControlStatus : public dc_measurements::Measurement
{
public:
  Ros2ControlStatus();
  ~Ros2ControlStatus() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  void activityCb(const controller_manager_msgs::msg::ControllerManagerActivity& msg);
  void processEntries(const std::vector<controller_manager_msgs::msg::NamedLifecycleState>& entries,
                      const std::string& component_type, const rclcpp::Time& stamp,
                      std::map<std::string, dc_common::StateTransitionDetector<uint8_t>>& detectors);

  rclcpp::Subscription<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr subscription_;
  std::string topic_;

  // The activity callback and the polling timer run in different callback groups under a
  // multi-threaded executor, so everything they share is guarded.
  mutable std::mutex mutex_;
  std::map<std::string, dc_common::StateTransitionDetector<uint8_t>> controller_detectors_;
  std::map<std::string, dc_common::StateTransitionDetector<uint8_t>> hardware_component_detectors_;
  // Records wait here for a poll to carry them out, one per poll, so they travel the same publish
  // path (Conditions, buffering, Group) as every other Record.
  PendingRecordQueue pending_records_;

  // Per-Record, not per-component: a global counter across every controller and hardware
  // component, so a consumer can tell a dropped Record apart from one that never changed.
  std::uint64_t record_seq_{ 0 };

protected:
  void onConfigure() override;
  void onCleanup() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__ROS2_CONTROL_STATUS_HPP_
