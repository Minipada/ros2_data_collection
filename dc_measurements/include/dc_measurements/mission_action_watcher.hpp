// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__MISSION_ACTION_WATCHER_HPP_
#define DC_MEASUREMENTS__MISSION_ACTION_WATCHER_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/mission_outcome.hpp"
#include "dc_measurements/mission_record_json.hpp"
#include "dc_measurements/pending_record_queue.hpp"
#include "dc_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace dc_measurements
{

/// The GoalStatus values meaning a goal is under way, versus the triplet a goal can end on.
inline bool isActiveActionStatus(std::int8_t status)
{
  return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
         status == action_msgs::msg::GoalStatus::STATUS_CANCELING;
}

inline bool isTerminalActionStatus(std::int8_t status)
{
  return status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
         status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
         status == action_msgs::msg::GoalStatus::STATUS_ABORTED;
}

/// Terminal Results that carry a missed-waypoint list (FollowWaypoints) feed the extra payload
/// from noteResult(); Results without one (NavigateToPose) feed it from noteFeedback() instead.
template <typename ResponseT, class = void>
struct ResponseCarriesMissedWaypoints : std::false_type
{
};

template <typename ResponseT>
struct ResponseCarriesMissedWaypoints<ResponseT,
                                      std::void_t<decltype(std::declval<const ResponseT&>().result.missed_waypoints)>>
  : std::true_type
{
};

/**
 * @class dc_measurements::SingleGoalMissionPolicy
 * @brief The outcome-core policy for nav2 actions an action server handles one goal at a time
 * (#387/#389): lifts MissionNav2Tracker / MissionFollowWaypointsTracker -- themselves built on
 * dc_common::StateTransitionDetector -- to the protocol MissionActionWatcher drives.
 *
 * `ResultExtraT` is what the tracker's endMission() takes as the terminal Result's extra payload,
 * and decides which callback feeds it: NavigateToPose carries `std::optional<int>` recoveries,
 * captured from Feedback while a mission is open (noteFeedback); FollowWaypoints carries the
 * missed-waypoint list, extracted from the Result itself (noteResult). A policy whose extra does
 * not come from Feedback must therefore set kWatchFeedback = false.
 */
template <typename ActionT, typename TrackerT, typename ResultExtraT>
class SingleGoalMissionPolicy
{
public:
  using TimePoint = std::chrono::system_clock::time_point;

  void seed(TimePoint at)
  {
    tracker_.emplace(at);
  }

  /// Only an active status can start a mission: a goal whose first-ever sighting is already
  /// terminal leaves no trace -- there is no observed interval to report.
  std::optional<MissionStartFact> observe(const std::string& goal_id, std::int8_t status, TimePoint at)
  {
    if (!isActiveActionStatus(status))
    {
      return std::nullopt;
    }
    return tracker_->startMission(goal_id, at);
  }

  /// A refused active status warns only when a different goal is the one being tracked; repeat
  /// sightings of the already-tracked goal stay silent.
  bool busyWithAnotherMission(const std::string& goal_id) const
  {
    const auto active = activeMissionId();
    return active.has_value() && *active != goal_id;
  }

  /// Only the tracked goal's terminal status fetches a Result, once per mission -- a terminal
  /// status repeated across several status-array publishes fires the service call only once.
  bool shouldRequestResult(const std::string& goal_id)
  {
    const auto active = activeMissionId();
    if (!active.has_value() || *active != goal_id)
    {
      return false;
    }
    return result_requested_.insert(goal_id).second;
  }

  /// The service was unreachable for this goal -- allow the next terminal sighting to retry.
  void noteResultUnavailable(const std::string& goal_id)
  {
    result_requested_.erase(goal_id);
  }

  void noteFeedback(const std::string& goal_id, int number_of_recoveries)
  {
    (void)goal_id;
    if (tracker_.has_value() && tracker_->activeMissionId().has_value())
    {
      result_extra_ = number_of_recoveries;
    }
  }

  void noteResult(const typename ActionT::Impl::GetResultService::Response& response)
  {
    if constexpr (ResponseCarriesMissedWaypoints<typename ActionT::Impl::GetResultService::Response>::value)
    {
      result_extra_.clear();
      for (const auto& waypoint : response.result.missed_waypoints)
      {
        result_extra_.push_back({ waypoint.index, waypoint.error_code });
      }
    }
  }

  auto end(const std::string& goal_id, std::int8_t status, std::uint16_t error_code, std::string error_msg, TimePoint at)
  {
    auto extra = std::move(result_extra_);
    result_extra_ = {};
    result_requested_.erase(goal_id);
    return tracker_->endMission(goal_id, terminalStatusOf(status), error_code, std::move(error_msg), std::move(extra),
                                at);
  }

  std::vector<std::string> openMissionIds() const
  {
    const auto active = activeMissionId();
    return active.has_value() ? std::vector<std::string>{ *active } : std::vector<std::string>{};
  }

private:
  std::optional<std::string> activeMissionId() const
  {
    return tracker_.has_value() ? tracker_->activeMissionId() : std::nullopt;
  }

  static MissionTerminalStatus terminalStatusOf(std::int8_t status)
  {
    switch (status)
    {
      case action_msgs::msg::GoalStatus::STATUS_CANCELED:
        return MissionTerminalStatus::Canceled;
      case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        return MissionTerminalStatus::Aborted;
      default:
        return MissionTerminalStatus::Succeeded;
    }
  }

  std::optional<TrackerT> tracker_;
  std::set<std::string> result_requested_;
  ResultExtraT result_extra_{};
};

/**
 * @class dc_measurements::MissionActionWatcher
 * @brief The action-lifecycle adapter behind the nav2 Mission Measurements (#503): #387/#388/
 * #389's three hand-copied shells collapsed into one body, parameterized by action type and
 * outcome-core policy. Watches an action's `_action/status` (and, when the policy says so,
 * `_action/feedback`) topics and calls its `_action/get_result` service for the terminal Result,
 * emitting mission_start/mission_end Records (#305/ADR-0010) via PendingRecordQueue -- one Record
 * leaves per poll, the same publish path (Conditions, buffering, Group) as every other Record.
 *
 * A passive watcher, not a commander: the action is driven by whatever already dispatches missions
 * on the robot (the BT navigator, a fleet orchestrator, an operator command) -- DC never sends a
 * goal of its own, and uses the standard, public per-action topics/services every
 * `rclcpp_action::Server` exposes rather than `rclcpp_action::Client`'s typed API, which only
 * reports on goals the client itself sent.
 *
 * `PolicyT` carries what genuinely differs per action (see SingleGoalMissionPolicy and
 * MissionNav2ThroughPosesPolicy): the outcome core and how its facts map onto Records, the
 * mission_type and default action_name, the goal-id -> mission_id format (an already-shipped
 * Record-shape detail downstream consumers may depend on), whether Feedback is watched at all,
 * and the shutdown warning's wording. The tested cores themselves do not move.
 */
template <typename ActionT, typename PolicyT>
class MissionActionWatcher : public dc_measurements::Measurement
{
public:
  using GetResultService = typename ActionT::Impl::GetResultService;
  using FeedbackMsg = typename ActionT::Impl::FeedbackMessage;

  MissionActionWatcher() = default;
  ~MissionActionWatcher() override = default;

  dc_interfaces::msg::StringStamped collect() override;

protected:
  void onConfigure() override;
  void onCleanup() override;

private:
  void statusCb(const action_msgs::msg::GoalStatusArray& msg);
  void feedbackCb(const FeedbackMsg& msg);
  void requestResult(const unique_identifier_msgs::msg::UUID& uuid, const std::string& goal_id, std::int8_t status,
                     const rclcpp::Time& terminal_at);
  void handleResult(const std::string& goal_id, std::int8_t status, const rclcpp::Time& terminal_at,
                    const typename GetResultService::Response::SharedPtr& response);
  // Caller holds mutex_.
  void enqueue(json data, const rclcpp::Time& stamp);

  std::string action_name_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  typename rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
  typename rclcpp::Client<GetResultService>::SharedPtr result_client_;

  // The status/feedback callbacks and the get_result response callback run on different callback
  // groups than the polling timer under a multi-threaded executor, so everything they share is
  // guarded.
  mutable std::mutex mutex_;
  PolicyT policy_;
  PendingRecordQueue pending_records_;
};

template <typename ActionT, typename PolicyT>
void MissionActionWatcher<ActionT, PolicyT>::onConfigure()
{
  auto node = getNode();
  action_name_ = dc_util::get_str_type_param(node, measurement_name_, "action_name", PolicyT::kDefaultActionName);

  const rclcpp::Time now = node->get_clock()->now();
  policy_.seed(std::chrono::system_clock::time_point(std::chrono::nanoseconds(now.nanoseconds())));

  // Matches the QoS an action server publishes its status topic with (reliable, transient_local):
  // a late-joining watcher still gets the current goal's last-known status rather than waiting for
  // the next change.
  status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
      action_name_ + "/_action/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&MissionActionWatcher::statusCb, this, std::placeholders::_1));
  if constexpr (PolicyT::kWatchFeedback)
  {
    feedback_sub_ = node->create_subscription<FeedbackMsg>(action_name_ + "/_action/feedback", rclcpp::QoS(10),
                                                           std::bind(&MissionActionWatcher::feedbackCb, this,
                                                                     std::placeholders::_1));
  }
  result_client_ = node->create_client<GetResultService>(action_name_ + "/_action/get_result");
}

template <typename ActionT, typename PolicyT>
void MissionActionWatcher<ActionT, PolicyT>::onCleanup()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& mission_id : policy_.openMissionIds())
  {
    RCLCPP_WARN_STREAM(logger_, measurement_name_ << ": collection stopped with mission '" << mission_id << "' still "
                                                  << PolicyT::kCleanupStill << "; " << PolicyT::kCleanupConsequence);
  }
  status_sub_.reset();
  feedback_sub_.reset();
  result_client_.reset();
}

template <typename ActionT, typename PolicyT>
void MissionActionWatcher<ActionT, PolicyT>::enqueue(json data, const rclcpp::Time& stamp)
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

template <typename ActionT, typename PolicyT>
void MissionActionWatcher<ActionT, PolicyT>::statusCb(const action_msgs::msg::GoalStatusArray& msg)
{
  const rclcpp::Time now = getNode()->get_clock()->now();
  const std::chrono::system_clock::time_point at{ std::chrono::nanoseconds(now.nanoseconds()) };

  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& entry : msg.status_list)
  {
    const std::string goal_id = PolicyT::goalKeyOf(entry.goal_info.goal_id);

    if (const auto start = policy_.observe(goal_id, entry.status, at); start.has_value())
    {
      enqueue(missionStartJson(start->mission_id, PolicyT::kMissionType, start->sequence), now);
    }
    else if (isActiveActionStatus(entry.status) && policy_.busyWithAnotherMission(goal_id))
    {
      RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                  "Measurement " << measurement_name_ << ": goal '" << goal_id << "' accepted on '"
                                                 << action_name_
                                                 << "' while another mission is already being tracked; ignoring it.");
    }

    if (!isTerminalActionStatus(entry.status) || !policy_.shouldRequestResult(goal_id))
    {
      continue;
    }
    requestResult(entry.goal_info.goal_id, goal_id, entry.status, now);
  }
}

template <typename ActionT, typename PolicyT>
void MissionActionWatcher<ActionT, PolicyT>::feedbackCb(const FeedbackMsg& msg)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  policy_.noteFeedback(PolicyT::goalKeyOf(msg.goal_id), msg.feedback.number_of_recoveries);
}

template <typename ActionT, typename PolicyT>
void MissionActionWatcher<ActionT, PolicyT>::requestResult(const unique_identifier_msgs::msg::UUID& uuid,
                                                           const std::string& goal_id, std::int8_t status,
                                                           const rclcpp::Time& terminal_at)
{
  if constexpr (PolicyT::kRetryResultFetch)
  {
    if (!result_client_->service_is_ready())
    {
      RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                  "Measurement "
                                      << measurement_name_ << ": '" << action_name_
                                      << "/_action/get_result' is not ready; will retry on the next terminal "
                                         "status for '"
                                      << goal_id << "'.");
      policy_.noteResultUnavailable(goal_id);
      return;
    }
  }

  auto request = std::make_shared<typename GetResultService::Request>();
  request->goal_id = uuid;
  result_client_->async_send_request(
      request, [this, goal_id, status, terminal_at](typename rclcpp::Client<GetResultService>::SharedFuture future) {
        handleResult(goal_id, status, terminal_at, future.get());
      });
}

template <typename ActionT, typename PolicyT>
void MissionActionWatcher<ActionT, PolicyT>::handleResult(const std::string& goal_id, std::int8_t status,
                                                          const rclcpp::Time& terminal_at,
                                                          const typename GetResultService::Response::SharedPtr& response)
{
  const std::chrono::system_clock::time_point at{ std::chrono::nanoseconds(terminal_at.nanoseconds()) };

  const std::lock_guard<std::mutex> lock(mutex_);
  policy_.noteResult(*response);
  const auto end = policy_.end(goal_id, status, response->result.error_code, response->result.error_msg, at);
  if (end.has_value())
  {
    json data = missionEndJsonBase(end->mission_id, PolicyT::kMissionType, end->sequence, end->outcome,
                                   end->duration_sec, end->reason, end->error_code);
    PolicyT::enrichRecord(data, *end);
    enqueue(std::move(data), terminal_at);
  }
}

template <typename ActionT, typename PolicyT>
dc_interfaces::msg::StringStamped MissionActionWatcher<ActionT, PolicyT>::collect()
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

#endif  // DC_MEASUREMENTS__MISSION_ACTION_WATCHER_HPP_
