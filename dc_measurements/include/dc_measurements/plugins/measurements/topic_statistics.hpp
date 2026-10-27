// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__TOPIC_STATISTICS_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__TOPIC_STATISTICS_HPP_

#include <string>

#include "dc_core/measurement.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_util/node_utils.hpp"
#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_point.hpp"

namespace dc_measurements
{

// One Record per statistics_msgs/MetricsMessage on /statistics, published as each message
// arrives rather than on the polling timer: the per-topic statistics the ROS graph itself
// reports, without subscribing to the measured topic.
class TopicStatistics : public dc_measurements::Measurement
{
public:
  TopicStatistics();
  ~TopicStatistics() override;
  json collect() override;

private:
  void metricsCb(const statistics_msgs::msg::MetricsMessage& msg);

  rclcpp::Subscription<statistics_msgs::msg::MetricsMessage>::SharedPtr subscription_;
  std::string topic_;

protected:
  void onConfigure() override;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__TOPIC_STATISTICS_HPP_
