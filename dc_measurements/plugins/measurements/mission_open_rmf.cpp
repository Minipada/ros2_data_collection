// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/mission_open_rmf.hpp"

#include <cstdint>

namespace dc_measurements
{

namespace
{

// `cancellation.labels`/`killed.labels` are string arrays ("a single value like `dashboard`, or a
// key-value pair like `app=dashboard`" per rmf_api_msgs' own schema) -- joined verbatim rather
// than picking just the first, so none of the source's own detail is silently dropped.
std::optional<std::string> joinLabels(const nlohmann::json& labels)
{
  if (!labels.is_array())
  {
    return std::nullopt;
  }
  std::string joined;
  for (const auto& label : labels)
  {
    if (!label.is_string())
    {
      continue;
    }
    if (!joined.empty())
    {
      joined += "; ";
    }
    joined += label.get<std::string>();
  }
  return joined.empty() ? std::nullopt : std::optional<std::string>(joined);
}

// Best-effort `reason`/`error_code` extraction from whichever part of `TaskState` Open-RMF
// actually populated for `sample.status` -- see MissionOpenRmfCore's TaskStateSample doc comment:
// unlike nav2's always-present `error_msg`, none of these are guaranteed present.
void fillReason(const nlohmann::json& message, TaskStateSample& sample)
{
  switch (sample.status)
  {
    case RmfTaskStatus::Failed:
    {
      const auto errors_it = message.find("dispatch");
      if (errors_it != message.end() && errors_it->is_object() && errors_it->contains("errors") &&
          (*errors_it)["errors"].is_array() && !(*errors_it)["errors"].empty())
      {
        const auto& error = (*errors_it)["errors"].front();
        const std::string detail = error.value("detail", std::string());
        const std::string category = error.value("category", std::string());
        if (!category.empty() && !detail.empty())
        {
          sample.reason = category + ": " + detail;
        }
        else if (!detail.empty())
        {
          sample.reason = detail;
        }
        else if (!category.empty())
        {
          sample.reason = category;
        }
        if (error.contains("code") && error["code"].is_number_integer())
        {
          sample.error_code = static_cast<std::uint16_t>(error["code"].get<int>());
        }
      }
      else if (message.contains("detail") && message["detail"].is_string())
      {
        sample.reason = message["detail"].get<std::string>();
      }
      break;
    }
    case RmfTaskStatus::Canceled:
      if (message.contains("cancellation") && message["cancellation"].is_object())
      {
        sample.reason = joinLabels(message["cancellation"].value("labels", nlohmann::json::array()));
      }
      break;
    case RmfTaskStatus::Killed:
      if (message.contains("killed") && message["killed"].is_object())
      {
        sample.reason = joinLabels(message["killed"].value("labels", nlohmann::json::array()));
      }
      break;
    default:
      // No reason field for `completed` (nothing to explain) or `skipped` (rmf_api_msgs has no
      // dedicated detail object for it, unlike cancellation/killed).
      break;
  }
}

}  // namespace

MissionOpenRmf::MissionOpenRmf() : dc_measurements::Measurement()
{
}

MissionOpenRmf::~MissionOpenRmf() = default;

void MissionOpenRmf::onConfigure()
{
  auto node = getNode();
  websocket_url_ = dc_util::get_str_type_param(node, measurement_name_, "websocket_url");
  const int initial_backoff_ms =
      dc_util::get_int_type_param(node, measurement_name_, "reconnect_initial_backoff_ms", 500);
  const int max_backoff_ms = dc_util::get_int_type_param(node, measurement_name_, "reconnect_max_backoff_ms", 30000);

  dc_common::WebSocketJsonClient::Options options;
  options.initial_backoff = std::chrono::milliseconds(initial_backoff_ms);
  options.max_backoff = std::chrono::milliseconds(max_backoff_ms);

  client_ = std::make_unique<dc_common::WebSocketJsonClient>(options);
  client_->onMessage([this](const nlohmann::json& message) { handleMessage(message); });
  client_->onError([this](const std::string& error_message) {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                measurement_name_ << ": websocket error: " << error_message);
  });
  client_->connect(websocket_url_);
}

void MissionOpenRmf::onCleanup()
{
  if (client_)
  {
    client_->disconnect();
    client_.reset();
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& mission_id : core_.openMissionIds())
  {
    RCLCPP_WARN_STREAM(logger_, measurement_name_ << ": collection stopped with mission '" << mission_id
                                                  << "' still running; no mission_end Record will be published");
  }
}

void MissionOpenRmf::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "mission_open_rmf.json");
  }
}

void MissionOpenRmf::emit(json data, const rclcpp::Time& stamp)
{
  // One Record leaves per poll, same overflow policy as MissionNav2ThroughPoses::emit().
  if (pending_records_.push(std::move(data), stamp))
  {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                "Measurement " << measurement_name_
                                               << ": mission Records are arriving faster than the polling interval "
                                                  "can report them; dropping the oldest.");
  }
}

void MissionOpenRmf::handleMessage(const nlohmann::json& message)
{
  if (!message.is_object())
  {
    return;
  }
  const auto booking_it = message.find("booking");
  if (booking_it == message.end() || !booking_it->is_object() || !booking_it->contains("id") ||
      !(*booking_it)["id"].is_string())
  {
    // Not a TaskState (or one missing the one field this Measurement cannot proceed without) --
    // ignored rather than throwing off the rest of the connection.
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                measurement_name_ << ": received a message with no booking.id; ignoring it");
    return;
  }

  TaskStateSample sample;
  sample.mission_id = (*booking_it)["id"].get<std::string>();
  sample.mission_type = message.value("category", std::string());
  sample.status = parseRmfTaskStatus(message.value("status", std::string()));

  if (message.contains("unix_millis_start_time") && message["unix_millis_start_time"].is_number_integer())
  {
    sample.unix_millis_start_time = message["unix_millis_start_time"].get<std::int64_t>();
  }
  if (message.contains("unix_millis_finish_time") && message["unix_millis_finish_time"].is_number_integer())
  {
    sample.unix_millis_finish_time = message["unix_millis_finish_time"].get<std::int64_t>();
  }
  fillReason(message, sample);

  const rclcpp::Time stamp = getNode()->get_clock()->now();
  const std::chrono::system_clock::time_point now{ std::chrono::nanoseconds(stamp.nanoseconds()) };

  MissionOpenRmfCore::ObserveResult observed;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    observed = core_.observe(sample, now);
  }

  if (observed.start.has_value())
  {
    json data = missionStartJson(observed.start->mission_id, sample.mission_type, observed.start->sequence);
    const std::lock_guard<std::mutex> lock(mutex_);
    emit(std::move(data), stamp);
  }

  if (observed.end.has_value())
  {
    json data =
        missionEndJsonBase(observed.end->mission_id, sample.mission_type, observed.end->sequence, observed.end->outcome,
                           observed.end->duration_sec, observed.end->reason, observed.end->error_code);
    const std::lock_guard<std::mutex> lock(mutex_);
    emit(std::move(data), stamp);
  }
}

dc_interfaces::msg::StringStamped MissionOpenRmf::collect()
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
PLUGINLIB_EXPORT_CLASS(dc_measurements::MissionOpenRmf, dc_core::Measurement)
