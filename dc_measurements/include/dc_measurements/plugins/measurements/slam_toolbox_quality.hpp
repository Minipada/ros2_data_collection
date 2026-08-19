// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SLAM_TOOLBOX_QUALITY_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SLAM_TOOLBOX_QUALITY_HPP_

#include <deque>
#include <mutex>
#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/msg/loop_closure_event.hpp"

namespace dc_measurements
{

/**
 * @class dc_measurements::SlamToolboxQuality
 * @brief Localization quality from two of slam_toolbox's native topics: `/pose` on the polling
 * interval for a `sample` Record carrying the pose covariance, and
 * `/slam_toolbox/loop_closure_event` for a single-shot `loop_closure` Record per occurrence. The
 * source event carries no data beyond its own arrival, so loop closures per hour and time since
 * the last one -- a localization-drift risk indicator -- come entirely from Record timing
 * downstream, the same way battery's charging sessions do (#394).
 */
class SlamToolboxQuality : public dc_measurements::Measurement
{
public:
  SlamToolboxQuality();
  ~SlamToolboxQuality() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  void poseCb(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
  void loopClosureCb(const slam_toolbox::msg::LoopClosureEvent& msg);
  json sampleRecord() const;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<slam_toolbox::msg::LoopClosureEvent>::SharedPtr loop_closure_subscription_;
  std::string pose_topic_;
  std::string loop_closure_topic_;

  // The pose callback and the polling timer run in different callback groups under a
  // multi-threaded executor, so everything they share is guarded.
  mutable std::mutex mutex_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
  bool has_pose_{ false };
  // Loop closures wait here for a poll to carry them out, one Record per poll, so they travel
  // the same publish path (Conditions, buffering, Group) as every other Record.
  std::deque<rclcpp::Time> pending_loop_closures_;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__SLAM_TOOLBOX_QUALITY_HPP_
