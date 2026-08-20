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
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"

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
 *
 * The loop-closure topic is subscribed generically (`create_generic_subscription`, the same
 * runtime-discovery mechanism dc_bridge's raw mode uses), not with a compile-time
 * `slam_toolbox::msg::LoopClosureEvent`: that message is declared on slam_toolbox's `ros2`
 * branch but isn't part of any released binary yet (verified against the actual
 * `ros-jazzy-slam-toolbox` package contents -- no `LoopClosureEvent` there, only in
 * unreleased source), so a compile-time dependency on it can't build against a real
 * installation today. A generic subscription needs no message headers at all: the content is
 * never decoded, since the Record only needs to know an occurrence happened, not what the
 * message carried. The topic's type is discovered once it appears on the graph, so this
 * Measurement still works whenever slam_toolbox does start publishing it, regardless of which
 * release adds it or what the message's fields end up being.
 */
class SlamToolboxQuality : public dc_measurements::Measurement
{
public:
  SlamToolboxQuality();
  ~SlamToolboxQuality() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  void poseCb(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
  // A generic subscription's callback only ever gets the raw serialized bytes; this
  // Measurement doesn't decode them; the message's mere arrival is the Record.
  void loopClosureCb(std::shared_ptr<const rclcpp::SerializedMessage> msg);
  // Looks for loop_closure_topic_ on the graph and subscribes once its type is known.
  // Retried on discovery_timer_ until it succeeds, so startup order with slam_toolbox
  // doesn't matter.
  void tryCreateLoopClosureSubscription();
  json sampleRecord() const;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
  rclcpp::GenericSubscription::SharedPtr loop_closure_subscription_;
  rclcpp::TimerBase::SharedPtr loop_closure_discovery_timer_;
  std::string pose_topic_;
  std::string loop_closure_topic_;

  // The pose/loop-closure callbacks and the polling timer run in different callback groups
  // under a multi-threaded executor, so everything they share is guarded.
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
