// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/mission_nav2.hpp"

#include "dc_measurements/mission_uuid.hpp"

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

MissionNav2::MissionNav2() : dc_measurements::Measurement()
{
}

MissionNav2::~MissionNav2() = default;

void MissionNav2::onConfigure()
{
  auto node = getNode();
  action_name_ = dc_util::get_str_type_param(node, measurement_name_, "action_name", "navigate_to_pose");

  const rclcpp::Time now = node->get_clock()->now();
  tracker_.emplace(std::chrono::system_clock::time_point(std::chrono::nanoseconds(now.nanoseconds())));

  // Matches the QoS an action server publishes its status topic with (reliable, transient_local):
  // a late-joining watcher still gets the current goal's last-known status rather than waiting for
  // the next change.
  status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
      action_name_ + "/_action/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&MissionNav2::statusCb, this, std::placeholders::_1));
  feedback_sub_ =
      node->create_subscription<FeedbackMsg>(action_name_ + "/_action/feedback", rclcpp::QoS(10),
                                             std::bind(&MissionNav2::feedbackCb, this, std::placeholders::_1));
  result_client_ = node->create_client<GetResultService>(action_name_ + "/_action/get_result");
}

void MissionNav2::onCleanup()
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
  feedback_sub_.reset();
  result_client_.reset();
}

void MissionNav2::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "mission_nav2.json");
  }
}

void MissionNav2::enqueue(json data, const rclcpp::Time& stamp)
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

void MissionNav2::statusCb(const action_msgs::msg::GoalStatusArray& msg)
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
        enqueue(missionStartJson(start->mission_id, "navigate_to_pose", start->sequence), now);
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

void MissionNav2::feedbackCb(const FeedbackMsg& msg)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (tracker_.has_value() && tracker_->activeMissionId().has_value())
  {
    last_recoveries_ = msg.feedback.number_of_recoveries;
  }
}

void MissionNav2::requestResult(const unique_identifier_msgs::msg::UUID& uuid, const std::string& goal_id,
                                const rclcpp::Time& terminal_at)
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

void MissionNav2::handleResultResponse(const std::string& goal_id, const rclcpp::Time& terminal_at,
                                       const GetResultService::Response::SharedPtr& response)
{
  const std::chrono::system_clock::time_point at{ std::chrono::nanoseconds(terminal_at.nanoseconds()) };

  const std::lock_guard<std::mutex> lock(mutex_);
  result_requested_.erase(goal_id);
  const std::optional<int> recoveries = last_recoveries_;
  last_recoveries_.reset();
  const auto end = tracker_->endMission(goal_id, terminalStatusOf(response->status), response->result.error_code,
                                        response->result.error_msg, recoveries, at);
  if (end.has_value())
  {
    json data = missionEndJsonBase(end->mission_id, "navigate_to_pose", end->sequence, end->outcome,
                                   end->duration_sec, end->reason, end->error_code);
    if (end->recoveries.has_value())
    {
      data["recoveries"] = *end->recoveries;
    }
    enqueue(std::move(data), terminal_at);
  }
}

dc_interfaces::msg::StringStamped MissionNav2::collect()
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
PLUGINLIB_EXPORT_CLASS(dc_measurements::MissionNav2, dc_core::Measurement)
