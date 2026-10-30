// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/topic_statistics.hpp"

#include <cmath>
#include <cstdint>
#include <map>

namespace dc_measurements
{
namespace
{

struct KnownStatistic
{
  uint8_t data_type;
  const char* field;
  // sample_count is a count carried in a float64 field; the Record types it as an integer.
  bool integral;
};

const KnownStatistic kStatistics[] = {
  { statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE, "average", false },
  { statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM, "minimum", false },
  { statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM, "maximum", false },
  { statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_STDDEV, "stddev", false },
  { statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT, "sample_count", true },
};

}  // namespace

TopicStatistics::TopicStatistics() : dc_measurements::Measurement()
{
}

TopicStatistics::~TopicStatistics() = default;

void TopicStatistics::onConfigure()
{
  auto node = getNode();
  topic_ = dc_util::get_str_type_param(node, measurement_name_, "topic", "/statistics");

  subscription_ = node->create_subscription<statistics_msgs::msg::MetricsMessage>(
      topic_, rclcpp::SystemDefaultsQoS(), std::bind(&TopicStatistics::metricsCb, this, std::placeholders::_1));
}

void TopicStatistics::metricsCb(const statistics_msgs::msg::MetricsMessage& msg)
{
  if (!core_.collectible())
  {
    return;
  }

  json record;
  record["measurement_source_name"] = msg.measurement_source_name;
  record["metrics_source"] = msg.metrics_source;
  record["unit"] = msg.unit;
  record["window_start"] = { { "sec", msg.window_start.sec }, { "nanosec", msg.window_start.nanosec } };
  record["window_stop"] = { { "sec", msg.window_stop.sec }, { "nanosec", msg.window_stop.nanosec } };

  std::map<uint8_t, double> points;
  for (const auto& point : msg.statistics)
  {
    points.emplace(point.data_type, point.data);
  }

  // Every known statistic is in the Record; null means the producer did not measure it — an
  // absent point, or a non-finite one (the "not measured" marker of rclcpp's built-in collector
  // and of pulse_bridge's metric layout).
  for (const auto& known : kStatistics)
  {
    const auto point = points.find(known.data_type);
    record[known.field] = point != points.end() && std::isfinite(point->second) ?
                              (known.integral ? json(static_cast<int64_t>(point->second)) : json(point->second)) :
                              json(nullptr);
  }

  publish(record);
}

json TopicStatistics::collect()
{
  return json{};
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::TopicStatistics, dc_core::Measurement)
