// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/fault.hpp"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace dc_measurements
{

namespace
{
constexpr size_t kMaxPendingRecords = 64;

double secondsOf(const std::chrono::system_clock::duration& d)
{
  return std::chrono::duration<double>(d).count();
}

std::string iso8601(const std::chrono::system_clock::time_point& tp)
{
  const auto secs = std::chrono::system_clock::to_time_t(tp);
  const auto us =
      std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch()) % std::chrono::seconds(1);
  std::tm tm{};
  ::gmtime_r(&secs, &tm);
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << '.' << std::setfill('0') << std::setw(6) << us.count() << 'Z';
  return ss.str();
}

std::string levelName(uint8_t level)
{
  switch (level)
  {
    case diagnostic_msgs::msg::DiagnosticStatus::OK:
      return "OK";
    case diagnostic_msgs::msg::DiagnosticStatus::WARN:
      return "WARN";
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
      return "ERROR";
    case diagnostic_msgs::msg::DiagnosticStatus::STALE:
      return "STALE";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

Fault::Fault() : dc_measurements::Measurement()
{
}

Fault::~Fault() = default;

void Fault::onConfigure()
{
  auto node = getNode();
  topic_ = dc_util::get_str_type_param(node, measurement_name_, "topic", "/diagnostics");
  names_ = dc_util::get_str_array_type_param(node, measurement_name_, "names", std::vector<std::string>{});

  subscription_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      topic_, rclcpp::SystemDefaultsQoS(), std::bind(&Fault::diagnosticsCb, this, std::placeholders::_1));
}

void Fault::onCleanup()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& [name, started_at] : fault_started_at_)
  {
    RCLCPP_WARN_STREAM(logger_, measurement_name_ << ": collection stopped with '" << name
                                                  << "' still faulted, open since " << iso8601(started_at)
                                                  << "; its last Record stays marked open, no duration is reported");
  }
}

void Fault::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "fault.json");
  }
}

bool Fault::isWatched(const std::string& name) const
{
  return names_.empty() || std::find(names_.begin(), names_.end(), name) != names_.end();
}

void Fault::diagnosticsCb(const diagnostic_msgs::msg::DiagnosticArray& msg)
{
  const rclcpp::Time stamp = getNode()->get_clock()->now();
  const std::chrono::system_clock::time_point now{ std::chrono::nanoseconds(stamp.nanoseconds()) };

  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& status : msg.status)
  {
    if (!isWatched(status.name))
    {
      continue;
    }

    const auto transition = detectors_[status.name].update(status.level, now);
    if (!transition.has_value())
    {
      continue;
    }

    const bool raises = transition->from == diagnostic_msgs::msg::DiagnosticStatus::OK;
    const bool clears = transition->to == diagnostic_msgs::msg::DiagnosticStatus::OK;

    if (raises)
    {
      fault_started_at_[status.name] = transition->at;
    }
    const auto started_it = fault_started_at_.find(status.name);

    json data;
    data["seq"] = ++record_seq_;
    data["component"] = status.name;
    data["event"] = raises ? "raise" : (clears ? "clear" : "change");
    data["from_level"] = levelName(transition->from);
    data["to_level"] = levelName(transition->to);
    data["previous_level_duration_s"] = secondsOf(transition->dwell);
    data["reason"] = status.message;
    data["state"] = clears ? "closed" : "open";
    if (started_it != fault_started_at_.end())
    {
      data["fault_started_at"] = iso8601(started_it->second);
      if (clears)
      {
        data["duration_s"] = secondsOf(transition->at - started_it->second);
      }
    }
    if (clears)
    {
      fault_started_at_.erase(status.name);
    }

    pending_records_.emplace_back(std::move(data), stamp);
  }

  // One Record leaves per poll, so a component (or set of components) flapping far faster than
  // the polling interval would otherwise queue without bound. The oldest goes first: the recent
  // transitions are the ones still worth reporting.
  while (pending_records_.size() > kMaxPendingRecords)
  {
    pending_records_.pop_front();
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                "Measurement " << measurement_name_
                                               << ": fault Records are arriving faster than the polling interval "
                                                  "can report them; dropping the oldest.");
  }
}

dc_interfaces::msg::StringStamped Fault::collect()
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
PLUGINLIB_EXPORT_CLASS(dc_measurements::Fault, dc_core::Measurement)
