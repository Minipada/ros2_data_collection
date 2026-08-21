// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/mission_nav2_through_poses.hpp"

#include "dc_measurements/mission_uuid.hpp"

namespace dc_measurements
{

MissionNav2ThroughPoses::MissionNav2ThroughPoses() : dc_measurements::Measurement()
{
}

MissionNav2ThroughPoses::~MissionNav2ThroughPoses() = default;

void MissionNav2ThroughPoses::onConfigure()
{
  auto node = getNode();
  action_name_ = dc_util::get_str_type_param(node, measurement_name_, "action_name", "navigate_through_poses");

  // Matches the QoS an action server publishes its status topic with (reliable, transient_local):
  // a late-joining watcher still gets the current goal's last-known status rather than waiting for
  // the next change -- same reasoning as MissionNav2/MissionNav2FollowWaypoints.
  status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
      action_name_ + "/_action/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&MissionNav2ThroughPoses::statusCb, this, std::placeholders::_1));
  feedback_sub_ = node->create_subscription<FeedbackMsg>(action_name_ + "/_action/feedback", rclcpp::QoS(10),
                                                         std::bind(&MissionNav2ThroughPoses::feedbackCb, this,
                                                                   std::placeholders::_1));
  get_result_client_ =
      node->create_client<GetResultSrv>(action_name_ + "/_action/get_result", rclcpp::ServicesQoS(), client_cb_group_);
}

void MissionNav2ThroughPoses::onCleanup()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& mission_id : core_.openMissionIds())
  {
    RCLCPP_WARN_STREAM(logger_, measurement_name_ << ": collection stopped with mission '" << mission_id
                                                  << "' still running; no mission_end Record will be published");
  }
}

void MissionNav2ThroughPoses::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "mission_nav2_through_poses.json");
  }
}

void MissionNav2ThroughPoses::emit(json data, const rclcpp::Time& stamp)
{
  // One Record leaves per poll, so missions starting/ending far faster than the polling interval
  // would otherwise queue without bound. The oldest goes first: the recent boundaries are the ones
  // still worth reporting.
  if (pending_records_.push(std::move(data), stamp))
  {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                "Measurement " << measurement_name_
                                               << ": mission Records are arriving faster than the polling interval "
                                                  "can report them; dropping the oldest.");
  }
}

void MissionNav2ThroughPoses::statusCb(const action_msgs::msg::GoalStatusArray& msg)
{
  const rclcpp::Time stamp = getNode()->get_clock()->now();
  const std::chrono::system_clock::time_point now{ std::chrono::nanoseconds(stamp.nanoseconds()) };

  for (const auto& status : msg.status_list)
  {
    const std::string goal_id_hex = missionGoalIdHex(status.goal_info.goal_id.uuid);

    std::optional<MissionStartFact> start;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      start = core_.observe(goal_id_hex, now);
    }
    if (start.has_value())
    {
      json data = missionStartJson(start->mission_id, "navigate_through_poses", start->sequence);
      const std::lock_guard<std::mutex> lock(mutex_);
      emit(std::move(data), stamp);
    }

    const auto phase = static_cast<GoalPhase>(status.status);
    if (phase == GoalPhase::Succeeded || phase == GoalPhase::Canceled || phase == GoalPhase::Aborted)
    {
      requestResult(goal_id_hex, status.goal_info.goal_id, phase, stamp);
    }
  }
}

void MissionNav2ThroughPoses::feedbackCb(const FeedbackMsg& msg)
{
  const std::string goal_id_hex = missionGoalIdHex(msg.goal_id.uuid);
  const std::lock_guard<std::mutex> lock(mutex_);
  last_recoveries_[goal_id_hex] = msg.feedback.number_of_recoveries;
}

void MissionNav2ThroughPoses::requestResult(const std::string& goal_id_hex,
                                            const unique_identifier_msgs::msg::UUID& goal_id, GoalPhase phase,
                                            const rclcpp::Time& detected_at)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!core_.shouldRequestResult(goal_id_hex))
    {
      return;
    }
  }

  auto request = std::make_shared<GetResultSrv::Request>();
  request->goal_id = goal_id;

  get_result_client_->async_send_request(request, [this, goal_id_hex, phase,
                                                   detected_at](rclcpp::Client<GetResultSrv>::SharedFuture future) {
    handleResult(goal_id_hex, phase, detected_at, future.get());
  });
}

void MissionNav2ThroughPoses::handleResult(const std::string& goal_id_hex, GoalPhase phase,
                                           const rclcpp::Time& detected_at,
                                           const GetResultSrv::Response::SharedPtr& response)
{
  const std::chrono::system_clock::time_point at{ std::chrono::nanoseconds(detected_at.nanoseconds()) };

  std::optional<int> recoveries;
  MissionEndFact fact;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    auto recoveries_it = last_recoveries_.find(goal_id_hex);
    if (recoveries_it != last_recoveries_.end())
    {
      recoveries = recoveries_it->second;
      last_recoveries_.erase(recoveries_it);
    }
    fact = core_.end(goal_id_hex, phase, response->result.error_code, response->result.error_msg, recoveries, at);
  }

  json data = missionEndJsonBase(fact.mission_id, "navigate_through_poses", fact.sequence, fact.outcome,
                                 fact.duration_sec, fact.reason, fact.error_code);
  if (fact.recoveries.has_value())
  {
    data["recoveries"] = *fact.recoveries;
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  emit(std::move(data), detected_at);
}

dc_interfaces::msg::StringStamped MissionNav2ThroughPoses::collect()
{
  dc_interfaces::msg::StringStamped msg;
  msg.group_key = group_key_;

  const std::lock_guard<std::mutex> lock(mutex_);
  if (pending_records_.empty())
  {
    return msg;
  }

  auto record = pending_records_.pop();
  msg.header.stamp = record.second;
  msg.data = record.first.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::MissionNav2ThroughPoses, dc_core::Measurement)
