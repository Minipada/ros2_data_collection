// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/slam_toolbox_quality.hpp"

namespace dc_measurements
{

namespace
{
constexpr size_t kMaxPendingLoopClosures = 64;
}  // namespace

SlamToolboxQuality::SlamToolboxQuality() : dc_measurements::Measurement()
{
}

SlamToolboxQuality::~SlamToolboxQuality() = default;

void SlamToolboxQuality::onConfigure()
{
  auto node = getNode();
  pose_topic_ = dc_util::get_str_type_param(node, measurement_name_, "pose_topic", "/pose");
  loop_closure_topic_ =
      dc_util::get_str_type_param(node, measurement_name_, "loop_closure_topic", "/slam_toolbox/loop_closure_event");

  pose_subscription_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::SensorDataQoS(), std::bind(&SlamToolboxQuality::poseCb, this, std::placeholders::_1));
  loop_closure_subscription_ = node->create_subscription<slam_toolbox::msg::LoopClosureEvent>(
      loop_closure_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&SlamToolboxQuality::loopClosureCb, this, std::placeholders::_1));
}

void SlamToolboxQuality::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "slam_toolbox_quality.json");
  }
}

void SlamToolboxQuality::poseCb(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  last_pose_ = msg;
  has_pose_ = true;
}

void SlamToolboxQuality::loopClosureCb(const slam_toolbox::msg::LoopClosureEvent& msg)
{
  // slam_toolbox/msg/LoopClosureEvent carries only its own stamp; the occurrence is the Record,
  // so this Measurement's own clock -- not the message's stamp -- is what pairs it with the
  // pose samples around it.
  (void)msg;
  const auto now = getNode()->get_clock()->now();

  const std::lock_guard<std::mutex> lock(mutex_);
  pending_loop_closures_.push_back(now);

  // One event leaves per poll, so loop closures arriving far faster than the polling interval
  // would otherwise queue without bound. The oldest goes first: the recent ones are the ones
  // still worth reporting.
  while (pending_loop_closures_.size() > kMaxPendingLoopClosures)
  {
    pending_loop_closures_.pop_front();
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                "Measurement " << measurement_name_
                                               << ": loop closures are arriving faster than the polling interval "
                                                  "can report them; dropping the oldest.");
  }
}

json SlamToolboxQuality::sampleRecord() const
{
  json data;
  data["event"] = "sample";
  data["x"] = last_pose_.pose.pose.position.x;
  data["y"] = last_pose_.pose.pose.position.y;

  tf2::Quaternion q(last_pose_.pose.pose.orientation.x, last_pose_.pose.pose.orientation.y,
                    last_pose_.pose.pose.orientation.z, last_pose_.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  data["yaw"] = yaw;

  // PoseWithCovariance's 6x6 row-major covariance: index 0 is x-x, 7 is y-y, 35 is yaw-yaw --
  // the diagonal terms that read as localization confidence on their own axis, the same ones
  // AMCL/robot_localization dashboards already chart.
  data["covariance_x"] = last_pose_.pose.covariance[0];
  data["covariance_y"] = last_pose_.pose.covariance[7];
  data["covariance_yaw"] = last_pose_.pose.covariance[35];
  return data;
}

dc_interfaces::msg::StringStamped SlamToolboxQuality::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.group_key = group_key_;

  const std::lock_guard<std::mutex> lock(mutex_);

  // A loop closure takes the poll it lands on: it is a fact about a moment, so it keeps the
  // timestamp of that moment rather than this poll's.
  if (!pending_loop_closures_.empty())
  {
    const auto stamp = pending_loop_closures_.front();
    pending_loop_closures_.pop_front();
    msg.header.stamp = stamp;
    json event;
    event["event"] = "loop_closure";
    msg.data = event.dump(-1, ' ', true);
    return msg;
  }

  // Nothing on /pose yet: report nothing rather than a Record with no localization data.
  if (!has_pose_)
  {
    return msg;
  }

  msg.header.stamp = node->get_clock()->now();
  msg.data = sampleRecord().dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::SlamToolboxQuality, dc_core::Measurement)
