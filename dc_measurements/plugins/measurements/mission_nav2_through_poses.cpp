// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/mission_nav2_through_poses.hpp"

#include <iomanip>
#include <sstream>

#include "nav2_msgs/msg/waypoint_status.hpp"

namespace dc_measurements
{

namespace
{

constexpr size_t kMaxPendingRecords = 64;

std::string uuidToHex(const std::array<uint8_t, 16>& uuid)
{
  std::ostringstream oss;
  for (auto byte : uuid)
  {
    oss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(byte);
  }
  return oss.str();
}

std::string outcomeName(MissionOutcome outcome)
{
  switch (outcome)
  {
    case MissionOutcome::Succeeded:
      return "succeeded";
    case MissionOutcome::Failed:
      return "failed";
    case MissionOutcome::Cancelled:
      return "cancelled";
    case MissionOutcome::Aborted:
    default:
      return "aborted";
  }
}

std::optional<WaypointStatusCounts> countWaypointStatuses(const std::vector<nav2_msgs::msg::WaypointStatus>& statuses)
{
  if (statuses.empty())
  {
    return std::nullopt;
  }
  WaypointStatusCounts counts;
  counts.total = static_cast<std::uint32_t>(statuses.size());
  for (const auto& status : statuses)
  {
    switch (status.waypoint_status)
    {
      case nav2_msgs::msg::WaypointStatus::COMPLETED:
        ++counts.completed;
        break;
      case nav2_msgs::msg::WaypointStatus::SKIPPED:
        ++counts.skipped;
        break;
      case nav2_msgs::msg::WaypointStatus::FAILED:
        ++counts.failed;
        break;
      default:
        break;
    }
  }
  return counts;
}

}  // namespace

MissionNav2ThroughPoses::MissionNav2ThroughPoses() : dc_measurements::Measurement()
{
}

MissionNav2ThroughPoses::~MissionNav2ThroughPoses() = default;

void MissionNav2ThroughPoses::onConfigure()
{
  auto node = getNode();
  action_name_ = dc_util::get_str_type_param(node, measurement_name_, "action_name", "navigate_through_poses");

  status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
      action_name_ + "/_action/status", rclcpp::QoS(10),
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
  pending_records_.emplace_back(std::move(data), stamp);
  // One Record leaves per poll, so missions starting/ending far faster than the polling interval
  // would otherwise queue without bound. The oldest goes first: the recent boundaries are the ones
  // still worth reporting.
  while (pending_records_.size() > kMaxPendingRecords)
  {
    pending_records_.pop_front();
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
    const std::string goal_id_hex = uuidToHex(status.goal_info.goal_id.uuid);

    std::optional<MissionStartFact> start;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      start = core_.observe(goal_id_hex, now);
    }
    if (start.has_value())
    {
      json data;
      data["event"] = "mission_start";
      data["mission_id"] = start->mission_id;
      data["mission_type"] = "navigate_through_poses";
      data["sequence"] = start->sequence;
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
  const std::string goal_id_hex = uuidToHex(msg.goal_id.uuid);
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
    fact = core_.end(goal_id_hex, phase, response->result.error_code, response->result.error_msg, recoveries,
                     countWaypointStatuses(response->result.waypoint_statuses), at);
  }

  json data;
  data["event"] = "mission_end";
  data["mission_id"] = fact.mission_id;
  data["mission_type"] = "navigate_through_poses";
  data["sequence"] = fact.sequence;
  data["outcome"] = outcomeName(fact.outcome);
  data["duration_sec"] = fact.duration_sec;
  if (fact.reason.has_value())
  {
    data["reason"] = *fact.reason;
  }
  if (fact.error_code.has_value())
  {
    data["error_code"] = *fact.error_code;
  }
  if (fact.recoveries.has_value())
  {
    data["recoveries"] = *fact.recoveries;
  }
  if (fact.waypoint_statuses.has_value())
  {
    data["waypoint_statuses"] = { { "total", fact.waypoint_statuses->total },
                                  { "completed", fact.waypoint_statuses->completed },
                                  { "skipped", fact.waypoint_statuses->skipped },
                                  { "failed", fact.waypoint_statuses->failed } };
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

  auto record = std::move(pending_records_.front());
  pending_records_.pop_front();
  msg.header.stamp = record.second;
  msg.data = record.first.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::MissionNav2ThroughPoses, dc_core::Measurement)
