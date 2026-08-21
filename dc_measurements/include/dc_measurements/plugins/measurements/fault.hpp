// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__FAULT_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__FAULT_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "dc_common/state_transition_detector.hpp"
#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/pending_record_queue.hpp"
#include "dc_util/node_utils.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_measurements
{

/**
 * @class dc_measurements::Fault
 * @brief One Record per diagnostic level change -- the transition `diagnostics`'s periodic
 * snapshots leave a consumer to reconstruct by scanning for where a value changed. Built on the
 * same dc_common::StateTransitionDetector as Intervention (#362), one detector per watched
 * component so components fault and recover independently (#365).
 */
class Fault : public dc_measurements::Measurement
{
public:
  Fault();
  ~Fault() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  void diagnosticsCb(const diagnostic_msgs::msg::DiagnosticArray& msg);
  bool isWatched(const std::string& name) const;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  std::string topic_;
  // Which DiagnosticStatus.name values to track; empty tracks every component seen, matching
  // Diagnostics' own `names` filter. Non-empty keeps one noisy, unwatched component from growing
  // the detector map or flooding the pipeline.
  std::vector<std::string> names_;

  // The DiagnosticArray callback and the polling timer run in different callback groups under a
  // multi-threaded executor, so everything they share is guarded.
  mutable std::mutex mutex_;
  std::map<std::string, dc_common::StateTransitionDetector<uint8_t>> detectors_;
  // When each component's currently open fault (level != OK) started; erased once it clears. A
  // component observed already faulted on its very first sample has no entry here -- the
  // detector treats that sample as a baseline, not a transition, so no start was ever seen.
  std::map<std::string, std::chrono::system_clock::time_point> fault_started_at_;
  // Records wait here for a poll to carry them out, one per poll, so they travel the same publish
  // path (Conditions, buffering, Group) as every other Record.
  PendingRecordQueue pending_records_;

  // Per-Record, not per-component: a global counter across every watched component, so a
  // consumer can tell a dropped Record apart from a component that never changed.
  std::uint64_t record_seq_{ 0 };

protected:
  void onConfigure() override;
  void onCleanup() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__FAULT_HPP_
