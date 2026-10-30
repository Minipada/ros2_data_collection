// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <limits>

#include "measurement_test_bench.hpp"
#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_point.hpp"
#include "statistics_msgs/msg/statistic_data_type.hpp"

class MeasurementTopicStatisticsTest : public MeasurementBench
{
protected:
  MeasurementTopicStatisticsTest() : MeasurementBench("topic_statistics")
  {
    metrics_pub_ =
        ms_node_->create_publisher<statistics_msgs::msg::MetricsMessage>("/statistics", rclcpp::SystemDefaultsQoS());
  }

  static statistics_msgs::msg::StatisticDataPoint makePoint(uint8_t data_type, double data)
  {
    statistics_msgs::msg::StatisticDataPoint point;
    point.data_type = data_type;
    point.data = data;
    return point;
  }

  void publishMetrics(const std::vector<statistics_msgs::msg::StatisticDataPoint>& points)
  {
    statistics_msgs::msg::MetricsMessage msg;
    msg.measurement_source_name = "topic_statistics_test_node";
    msg.metrics_source = "/cmd_vel";
    msg.unit = "ms";
    msg.window_start.sec = 1700000000;
    msg.window_start.nanosec = 100;
    msg.window_stop.sec = 1700000010;
    msg.window_stop.nanosec = 200;
    msg.statistics = points;
    metrics_pub_->publish(msg);
  }

  rclcpp::Publisher<statistics_msgs::msg::MetricsMessage>::SharedPtr metrics_pub_;
};

TEST_F(MeasurementTopicStatisticsTest, FlattensEveryStatisticIntoItsNamedField)
{
  ms_node_->declare_parameter("topic_statistics.plugin", std::string("dc_measurements/TopicStatistics"));
  ms_node_->declare_parameter("topic_statistics.group_key", std::string("topic_statistics"));
  ms_node_->declare_parameter("topic_statistics.topic_output", std::string("/dc/measurement/topic_statistics"));
  ms_node_->declare_parameter("topic_statistics.init_collect", false);

  startLifecycleNode();
  waitForSubscriber("/statistics");

  ASSERT_TRUE(spinUntil([&] {
    publishMetrics({ makePoint(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE, 1.0),
                     makePoint(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM, 2.0),
                     makePoint(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM, 3.0),
                     makePoint(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_STDDEV, 4.0),
                     makePoint(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT, 5.0) });
    return callback_active_;
  })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["measurement_source_name"], "topic_statistics_test_node");
  EXPECT_EQ(data_json_["metrics_source"], "/cmd_vel");
  EXPECT_EQ(data_json_["unit"], "ms");
  EXPECT_EQ(data_json_["window_start"]["sec"], 1700000000);
  EXPECT_EQ(data_json_["window_start"]["nanosec"], 100);
  EXPECT_EQ(data_json_["window_stop"]["sec"], 1700000010);
  EXPECT_EQ(data_json_["window_stop"]["nanosec"], 200);
  EXPECT_EQ(data_json_["average"], 1.0);
  EXPECT_EQ(data_json_["minimum"], 2.0);
  EXPECT_EQ(data_json_["maximum"], 3.0);
  EXPECT_EQ(data_json_["stddev"], 4.0);
  EXPECT_EQ(data_json_["sample_count"], 5);
  EXPECT_TRUE(data_json_["sample_count"].is_number_integer());
}

TEST_F(MeasurementTopicStatisticsTest, StatisticsTheProducerOmitsArriveAsNull)
{
  ms_node_->declare_parameter("topic_statistics.plugin", std::string("dc_measurements/TopicStatistics"));
  ms_node_->declare_parameter("topic_statistics.group_key", std::string("topic_statistics"));
  ms_node_->declare_parameter("topic_statistics.topic_output", std::string("/dc/measurement/topic_statistics"));
  ms_node_->declare_parameter("topic_statistics.init_collect", false);

  startLifecycleNode();
  waitForSubscriber("/statistics");

  ASSERT_TRUE(spinUntil([&] {
    publishMetrics({ makePoint(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM, 1.0) });
    return callback_active_;
  })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["maximum"], 1.0);
  EXPECT_TRUE(data_json_["average"].is_null());
  EXPECT_TRUE(data_json_["minimum"].is_null());
  EXPECT_TRUE(data_json_["stddev"].is_null());
  EXPECT_TRUE(data_json_["sample_count"].is_null());
}

TEST_F(MeasurementTopicStatisticsTest, NonFiniteStatisticsArriveAsNull)
{
  ms_node_->declare_parameter("topic_statistics.plugin", std::string("dc_measurements/TopicStatistics"));
  ms_node_->declare_parameter("topic_statistics.group_key", std::string("topic_statistics"));
  ms_node_->declare_parameter("topic_statistics.topic_output", std::string("/dc/measurement/topic_statistics"));
  ms_node_->declare_parameter("topic_statistics.init_collect", false);

  startLifecycleNode();
  waitForSubscriber("/statistics");

  ASSERT_TRUE(spinUntil([&] {
    publishMetrics({ makePoint(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE,
                               std::numeric_limits<double>::quiet_NaN()),
                     makePoint(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT, 7.0) });
    return callback_active_;
  })) << "no Record ever arrived";

  EXPECT_TRUE(data_json_["average"].is_null());
  EXPECT_EQ(data_json_["sample_count"], 7);
}

DC_MEASUREMENT_TEST_MAIN()
