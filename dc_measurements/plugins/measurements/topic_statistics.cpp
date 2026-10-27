// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_measurements/plugins/measurements/topic_statistics.hpp"

namespace dc_measurements
{
namespace
{

const char* statFieldName(uint8_t data_type)
{
  switch (data_type)
  {
    case statistics_msgs::msg::StatisticDataPoint::AVERAGE:
      return "average";
    case statistics_msgs::msg::StatisticDataPoint::MINIMUM:
      return "minimum";
    case statistics_msgs::msg::StatisticDataPoint::MAXIMUM:
      return "maximum";
    case statistics_msgs::msg::StatisticDataPoint::STDDEV:
      return "stddev";
    case statistics_msgs::msg::StatisticDataPoint::SAMPLE_COUNT:
      return "sample_count";
    default:
      return nullptr;
  }
}

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

  for (const auto& point : msg.statistics)
  {
    const char* field = statFieldName(point.data_type);
    if (field == nullptr)
    {
      continue;
    }
    // sample_count is a count carried in a float64 field; the Record types it as an integer.
    record[field] = point.data_type == statistics_msgs::msg::StatisticDataPoint::SAMPLE_COUNT ?
                        json(static_cast<int64_t>(point.data)) :
                        json(point.data);
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
