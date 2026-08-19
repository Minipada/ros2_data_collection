// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/manipulation.hpp"

#include <iomanip>
#include <sstream>

#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace dc_measurements
{

namespace
{

constexpr size_t kMaxPendingEvents = 64;

// Canonical 8-4-4-4-12 hex form, used both as this Record's `goal_id` and as the internal
// tracking key -- readable in a Record without a lookup table.
std::string uuidToString(const unique_identifier_msgs::msg::UUID& uuid)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < uuid.uuid.size(); ++i)
  {
    if (i == 4 || i == 6 || i == 8 || i == 10)
    {
      oss << '-';
    }
    oss << std::setw(2) << static_cast<int>(uuid.uuid[i]);
  }
  return oss.str();
}

double secondsOf(const rclcpp::Time& from, const rclcpp::Time& to)
{
  return to > from ? (to - from).seconds() : 0.0;
}

// error_code is MoveIt's own MoveItErrorCodes space, not Mission's nav2 outcome split (#393):
// SUCCESS succeeds, PREEMPTED is the closest thing MoveIt has to cancelled, and every other
// (necessarily negative) code is a failure -- no separate mapping table needed on DC's side.
std::string outcomeOf(int32_t error_code)
{
  if (error_code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    return "succeeded";
  }
  if (error_code == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED)
  {
    return "cancelled";
  }
  return "failed";
}

}  // namespace

Manipulation::Manipulation() : dc_measurements::Measurement()
{
}

Manipulation::~Manipulation() = default;

void Manipulation::onConfigure()
{
  auto node = getNode();
  action_name_ = dc_util::get_str_type_param(node, measurement_name_, "action_name", "move_action");
  // No sane default across robots/planning groups; must be configured.
  group_name_ = dc_util::get_str_type_param(node, measurement_name_, "group_name");

  status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
      action_name_ + "/_action/status", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&Manipulation::statusCb, this, std::placeholders::_1));
  get_result_client_ =
      node->create_client<GetResult>(action_name_ + "/_action/get_result", rclcpp::ServicesQoS(), client_cb_group_);
}

void Manipulation::onCleanup()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& [key, tracking] : goals_)
  {
    RCLCPP_WARN_STREAM(logger_, measurement_name_
                                    << ": collection stopped with goal '" << key
                                    << "' still unresolved; no manipulation_end Record will be published");
  }
}

void Manipulation::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "manipulation.json");
  }
}

void Manipulation::statusCb(const action_msgs::msg::GoalStatusArray& msg)
{
  const rclcpp::Time stamp = getNode()->get_clock()->now();

  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& status : msg.status_list)
  {
    const std::string key = uuidToString(status.goal_info.goal_id);
    const bool is_terminal = status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
                             status.status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
                             status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED;

    auto it = goals_.find(key);
    if (it == goals_.end())
    {
      if (is_terminal)
      {
        // Either a stale republish of a goal this instance already resolved and stopped
        // tracking, or a goal that reached a terminal state before this instance ever saw it
        // accepted -- neither can be honestly reported with a start it never observed.
        continue;
      }

      goals_.emplace(key, GoalTracking{ stamp, GoalState::kOpen });

      json data;
      data["event"] = "manipulation_start";
      data["goal_id"] = key;
      data["group_name"] = group_name_;
      data["sequence"] = ++record_seq_;
      pending_events_.emplace_back(std::move(data), stamp);
      continue;
    }

    if (it->second.state != GoalState::kOpen || !is_terminal)
    {
      // Already awaiting a result, or a non-terminal update (e.g. CANCELING) on a goal already
      // being tracked -- nothing new to report yet.
      continue;
    }

    const rclcpp::Time accepted_stamp = it->second.accepted_stamp;
    it->second.state = GoalState::kResultPending;

    auto request = std::make_shared<GetResult::Request>();
    request->goal_id = status.goal_info.goal_id;
    get_result_client_->async_send_request(request, [this, key, accepted_stamp,
                                                     stamp](rclcpp::Client<GetResult>::SharedFuture future) {
      onResult(key, accepted_stamp, stamp, future);
    });
  }
}

void Manipulation::onResult(const std::string& goal_key, const rclcpp::Time& accepted_stamp,
                            const rclcpp::Time& terminal_stamp, rclcpp::Client<GetResult>::SharedFuture future)
{
  const auto response = future.get();
  const int32_t error_code = response->result.error_code.val;
  enqueueEnd(goal_key, outcomeOf(error_code), error_code, response->result.planning_time,
             secondsOf(accepted_stamp, terminal_stamp), terminal_stamp);
}

void Manipulation::enqueueEnd(const std::string& goal_key, const std::string& outcome, int32_t error_code,
                              double planning_time, double duration_sec, const rclcpp::Time& stamp)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  // Erased, not marked resolved: see the class-level comment on `goals_` for why an unknown key
  // is exactly how a later stale status republish for this same goal is recognized and skipped.
  goals_.erase(goal_key);

  json data;
  data["event"] = "manipulation_end";
  data["goal_id"] = goal_key;
  data["group_name"] = group_name_;
  data["sequence"] = ++record_seq_;
  data["outcome"] = outcome;
  data["error_code"] = error_code;
  data["planning_time"] = planning_time;
  data["duration_sec"] = duration_sec;
  pending_events_.emplace_back(std::move(data), stamp);

  // One event leaves per poll, so goals resolving far faster than the polling interval would
  // otherwise queue without bound. The oldest goes first: the recent boundaries are the ones
  // still worth reporting.
  while (pending_events_.size() > kMaxPendingEvents)
  {
    pending_events_.pop_front();
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                "Measurement " << measurement_name_
                                               << ": manipulation Records are arriving faster than the polling "
                                                  "interval can report them; dropping the oldest.");
  }
}

dc_interfaces::msg::StringStamped Manipulation::collect()
{
  dc_interfaces::msg::StringStamped msg;
  msg.group_key = group_key_;

  const std::lock_guard<std::mutex> lock(mutex_);
  if (pending_events_.empty())
  {
    return msg;
  }

  auto event = std::move(pending_events_.front());
  pending_events_.pop_front();
  msg.header.stamp = event.second;
  msg.data = event.first.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Manipulation, dc_core::Measurement)
