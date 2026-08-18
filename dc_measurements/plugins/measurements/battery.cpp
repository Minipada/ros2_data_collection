// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/battery.hpp"

#include <cmath>

namespace dc_measurements
{

namespace
{
constexpr size_t kMaxPendingEvents = 64;

// sensor_msgs/BatteryState leaves most fields optional and signals "unmeasured" with NaN, so a
// field the hardware doesn't fill is left out of the Record rather than written as null.
void setIfMeasured(json& data, const std::string& key, float value)
{
  if (!std::isnan(value))
  {
    data[key] = value;
  }
}

std::string statusName(uint8_t status)
{
  switch (status)
  {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
      return "charging";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
      return "discharging";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
      return "not_charging";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL:
      return "full";
    default:
      return "unknown";
  }
}

std::string healthName(uint8_t health)
{
  switch (health)
  {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD:
      return "good";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT:
      return "overheat";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD:
      return "dead";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE:
      return "overvoltage";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:
      return "unspecified_failure";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_COLD:
      return "cold";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE:
      return "watchdog_timer_expire";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE:
      return "safety_timer_expire";
    default:
      return "unknown";
  }
}

std::string technologyName(uint8_t technology)
{
  switch (technology)
  {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH:
      return "nimh";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION:
      return "lion";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO:
      return "lipo";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE:
      return "life";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NICD:
      return "nicd";
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIMN:
      return "limn";
    default:
      return "unknown";
  }
}

void setIfPresent(json& data, const std::string& key, const std::optional<double>& value)
{
  if (value)
  {
    data[key] = *value;
  }
}

double toSeconds(dc_common::ChargingSession::Duration duration)
{
  return std::chrono::duration<double>(duration).count();
}
}  // namespace

Battery::Battery() : dc_measurements::Measurement()
{
}

Battery::~Battery() = default;

void Battery::onConfigure()
{
  auto node = getNode();
  battery_topic_ = dc_util::get_str_type_param(node, measurement_name_, "topic", "/battery_state");
  // sensor_msgs/BatteryState specifies percentage on a 0-1 range; drivers that already publish
  // 0-100 are configured with a scale of 1.0.
  percentage_scale_ = dc_util::get_double_type_param(node, measurement_name_, "percentage_scale", 100.0);

  subscription_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_, rclcpp::SensorDataQoS(), std::bind(&Battery::batteryStateCb, this, std::placeholders::_1));
}

void Battery::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "battery.json");
  }
}

void Battery::batteryStateCb(const sensor_msgs::msg::BatteryState& msg)
{
  const auto now = getNode()->get_clock()->now();
  const auto stamp = dc_common::BatteryCycleAccumulator::TimePoint(std::chrono::nanoseconds(now.nanoseconds()));

  const std::lock_guard<std::mutex> lock(mutex_);
  last_msg_ = msg;
  has_sample_ = true;

  std::optional<double> percentage;
  if (!std::isnan(msg.percentage))
  {
    percentage = msg.percentage * percentage_scale_;
  }

  const auto update =
      accumulator_.update(percentage, static_cast<dc_common::PowerSupplyStatus>(msg.power_supply_status), stamp);

  if (update.started)
  {
    const auto& session = *update.started;
    json event;
    event["event"] = "charge_session_start";
    event["session_id"] = session.sequence;
    event["discharge_depth_percent"] = session.preceding_discharge_depth;
    setIfPresent(event, "percentage", session.start_percentage);
    pending_events_.emplace_back(std::move(event), now);
  }
  if (update.ended)
  {
    const auto& session = *update.ended;
    json event;
    event["event"] = "charge_session_end";
    event["session_id"] = session.sequence;
    event["duration_sec"] = toSeconds(session.duration(stamp));
    setIfPresent(event, "start_percentage", session.start_percentage);
    setIfPresent(event, "end_percentage", session.end_percentage);
    if (session.start_percentage && session.end_percentage)
    {
      event["charged_percent"] = *session.end_percentage - *session.start_percentage;
    }
    pending_events_.emplace_back(std::move(event), now);
  }

  // One event leaves per poll, so a pack whose status flaps far faster than the polling interval
  // would otherwise queue without bound. The oldest goes first: the recent boundaries are the
  // ones still worth reporting.
  while (pending_events_.size() > kMaxPendingEvents)
  {
    pending_events_.pop_front();
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *getNode()->get_clock(), 10000,
                                "Measurement " << measurement_name_
                                               << ": charging session boundaries are arriving faster than the "
                                                  "polling interval can report them; dropping the oldest.");
  }
}

json Battery::sampleRecord() const
{
  json data;
  data["event"] = "sample";
  data["power_supply_status"] = statusName(last_msg_.power_supply_status);
  data["present"] = last_msg_.present;

  if (!std::isnan(last_msg_.percentage))
  {
    data["percentage"] = last_msg_.percentage * percentage_scale_;
  }
  setIfMeasured(data, "voltage", last_msg_.voltage);
  setIfMeasured(data, "current", last_msg_.current);
  setIfMeasured(data, "charge", last_msg_.charge);
  setIfMeasured(data, "capacity", last_msg_.capacity);
  setIfMeasured(data, "design_capacity", last_msg_.design_capacity);
  setIfMeasured(data, "temperature", last_msg_.temperature);

  if (last_msg_.power_supply_health != sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN)
  {
    data["power_supply_health"] = healthName(last_msg_.power_supply_health);
  }
  if (last_msg_.power_supply_technology != sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
  {
    data["power_supply_technology"] = technologyName(last_msg_.power_supply_technology);
  }
  // State of health: what the pack still holds against what it was built to hold. Only the
  // hardware reporting both capacities can answer it.
  if (!std::isnan(last_msg_.capacity) && !std::isnan(last_msg_.design_capacity) && last_msg_.design_capacity > 0.0F)
  {
    data["health_percentage"] = 100.0 * last_msg_.capacity / last_msg_.design_capacity;
  }
  if (!last_msg_.location.empty())
  {
    data["location"] = last_msg_.location;
  }
  if (!last_msg_.serial_number.empty())
  {
    data["serial_number"] = last_msg_.serial_number;
  }

  data["completed_cycles"] = accumulator_.completedCycles();
  if (const auto& session = accumulator_.openSession())
  {
    data["session_id"] = session->sequence;
  }
  return data;
}

dc_interfaces::msg::StringStamped Battery::collect()
{
  auto node = getNode();
  dc_interfaces::msg::StringStamped msg;
  msg.group_key = group_key_;

  const std::lock_guard<std::mutex> lock(mutex_);

  // A session boundary takes the poll it lands on: it is a fact about a moment, so it keeps the
  // timestamp of that moment rather than this poll's.
  if (!pending_events_.empty())
  {
    auto event = std::move(pending_events_.front());
    pending_events_.pop_front();
    msg.header.stamp = event.second;
    msg.data = event.first.dump(-1, ' ', true);
    return msg;
  }

  // Nothing on the topic yet: report nothing rather than a Record full of absent fields.
  if (!has_sample_)
  {
    return msg;
  }

  msg.header.stamp = node->get_clock()->now();
  msg.data = sampleRecord().dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Battery, dc_core::Measurement)
