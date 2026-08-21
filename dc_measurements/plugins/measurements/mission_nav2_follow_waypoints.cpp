// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/mission_nav2_follow_waypoints.hpp"

#include "dc_measurements/mission_uuid.hpp"
#include "nav2_msgs/msg/missed_waypoint.hpp"

namespace dc_measurements
{

namespace
{

MissionTerminalStatus terminalStatusOf(int8_t action_status)
{
  switch (action_status)
  {
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      return MissionTerminalStatus::Canceled;
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      return MissionTerminalStatus::Aborted;
    default:
      return MissionTerminalStatus::Succeeded;
  }
}

}  // namespace

MissionNav2FollowWaypoints::MissionNav2FollowWaypoints() : dc_measurements::Measurement()
{
}

MissionNav2FollowWaypoints::~MissionNav2FollowWaypoints() = default;

void MissionNav2FollowWaypoints::onConfigure()
{
  auto node = getNode();
  action_name_ = dc_util::get_str_type_param(node, measurement_name_, "action_name", "follow_waypoints");

  const rclcpp::Time now = node->get_clock()->now();
  tracker_.emplace(std::chrono::system_clock::time_point(std::chrono::nanoseconds(now.nanoseconds())));

  // Matches the QoS an action server publishes its status topic with (reliable, transient_local):
  // a late-joining watcher still gets the current goal's last-known status rather than waiting for
  // the next change.
  status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
      action_name_ + "/_action/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&MissionNav2FollowWaypoints::statusCb, this, std::placeholders::_1));
  result_client_ = node->create_client<GetResultService>(action_name_ + "/_action/get_result");
}

void MissionNav2FollowWaypoints::onCleanup()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (tracker_.has_value())
  {
    if (const auto active = tracker_->activeMissionId())
    {
      RCLCPP_WARN_STREAM(logger_, measurement_name_ << ": collection stopped with mission '" << *active
                                                    << "' still active; no closing Record will be published");
    }
  }
  status_sub_.reset();
  result_client_.reset();
}

void MissionNav2FollowWaypoints::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "mission_nav2_follow_waypoints.json");
  }
}

void MissionNav2FollowWaypoints::enqueue(json data, const rclcpp::Time& stamp)
{
  if (pending_records_.push(std::move(data), stamp))
  {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                "Measurement " << measurement_name_
                                               << ": mission Records are arriving faster than the polling interval "
                                                  "can report them; dropping the oldest.");
  }
}

void MissionNav2FollowWaypoints::statusCb(const action_msgs::msg::GoalStatusArray& msg)
{
  const rclcpp::Time now = getNode()->get_clock()->now();
  const std::chrono::system_clock::time_point at{ std::chrono::nanoseconds(now.nanoseconds()) };

  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& entry : msg.status_list)
  {
    const std::string goal_id = missionGoalIdDashed(entry.goal_info.goal_id);

    if (entry.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        entry.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        entry.status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
    {
      const auto start = tracker_->startMission(goal_id, at);
      if (start.has_value())
      {
        enqueue(missionStartJson(start->mission_id, "follow_waypoints", start->sequence), now);
      }
      else if (!tracker_->activeMissionId().has_value() || *tracker_->activeMissionId() != goal_id)
      {
        RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                    "Measurement " << measurement_name_ << ": goal '" << goal_id << "' accepted on '"
                                                   << action_name_
                                                   << "' while another mission is already being tracked; ignoring it.");
      }
      continue;
    }

    const bool is_terminal = entry.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
                             entry.status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
                             entry.status == action_msgs::msg::GoalStatus::STATUS_ABORTED;
    if (!is_terminal)
    {
      continue;
    }
    if (!tracker_->activeMissionId().has_value() || *tracker_->activeMissionId() != goal_id)
    {
      // Not the mission this instance is following (or already closed) -- most likely this same
      // terminal status republished after its get_result response already ended it.
      continue;
    }
    if (!result_requested_.insert(goal_id).second)
    {
      continue;
    }
    requestResult(entry.goal_info.goal_id, goal_id, now);
  }
}

void MissionNav2FollowWaypoints::requestResult(const unique_identifier_msgs::msg::UUID& uuid,
                                               const std::string& goal_id, const rclcpp::Time& terminal_at)
{
  if (!result_client_->service_is_ready())
  {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                "Measurement " << measurement_name_ << ": '" << action_name_
                                               << "/_action/get_result' is not ready; will retry on the next terminal "
                                                  "status for '"
                                               << goal_id << "'.");
    result_requested_.erase(goal_id);
    return;
  }

  auto request = std::make_shared<GetResultService::Request>();
  request->goal_id = uuid;
  result_client_->async_send_request(request, [this, goal_id,
                                               terminal_at](rclcpp::Client<GetResultService>::SharedFuture future) {
    handleResultResponse(goal_id, terminal_at, future.get());
  });
}

void MissionNav2FollowWaypoints::handleResultResponse(const std::string& goal_id, const rclcpp::Time& terminal_at,
                                                      const GetResultService::Response::SharedPtr& response)
{
  std::vector<WaypointOutcome> missed;
  missed.reserve(response->result.missed_waypoints.size());
  for (const auto& waypoint : response->result.missed_waypoints)
  {
    missed.push_back({ waypoint.index, waypoint.error_code });
  }

  const std::chrono::system_clock::time_point at{ std::chrono::nanoseconds(terminal_at.nanoseconds()) };

  const std::lock_guard<std::mutex> lock(mutex_);
  result_requested_.erase(goal_id);
  const auto end = tracker_->endMission(goal_id, terminalStatusOf(response->status), response->result.error_code,
                                        response->result.error_msg, std::move(missed), at);
  if (end.has_value())
  {
    json data = missionEndJsonBase(end->mission_id, "follow_waypoints", end->sequence, end->outcome, end->duration_sec,
                                   end->reason, end->error_code);
    json missed_json = json::array();
    for (const auto& waypoint : end->missed_waypoints)
    {
      missed_json.push_back({ { "index", waypoint.index }, { "error_code", waypoint.error_code } });
    }
    data["missed_waypoints"] = missed_json;
    enqueue(std::move(data), terminal_at);
  }
}

dc_interfaces::msg::StringStamped MissionNav2FollowWaypoints::collect()
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
PLUGINLIB_EXPORT_CLASS(dc_measurements::MissionNav2FollowWaypoints, dc_core::Measurement)
