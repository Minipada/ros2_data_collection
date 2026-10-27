// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "measurement_test_bench.hpp"
#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_point.hpp"

class MeasurementTopicStatisticsTest : public MeasurementBench
{
protected:
  MeasurementTopicStatisticsTest() : MeasurementBench("topic_statistics")
  {
    metrics_pub_ =
        ms_node_->create_publisher<statistics_msgs::msg::MetricsMessage>("/statistics", rclcpp::SystemDefaultsQoS());
  }

  static statistics_msgs::msg::MetricsMessage makeMetrics(const std::vector<uint8_t>& data_types)
  {
    statistics_msgs::msg::MetricsMessage msg;
    msg.measurement_source_name = "topic_statistics_test_node";
    msg.metrics_source = "/cmd_vel";
    msg.unit = "ms";
    msg.window_start.sec = 1700000000;
    msg.window_start.nanosec = 100;
    msg.window_stop.sec = 1700000010;
    msg.window_stop.nanosec = 200;

    for (size_t index = 0; index < data_types.size(); index++)
    {
      statistics_msgs::msg::StatisticDataPoint point;
      point.data_type = data_types[index];
      point.data = static_cast<double>(index + 1);
      msg.statistics.push_back(point);
    }

    return msg;
  }

  void publishMetrics(const std::vector<uint8_t>& data_types)
  {
    metrics_pub_->publish(makeMetrics(data_types));
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
    publishMetrics(
        { statistics_msgs::msg::StatisticDataPoint::AVERAGE, statistics_msgs::msg::StatisticDataPoint::MINIMUM,
          statistics_msgs::msg::StatisticDataPoint::MAXIMUM, statistics_msgs::msg::StatisticDataPoint::STDDEV,
          statistics_msgs::msg::StatisticDataPoint::SAMPLE_COUNT });
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

TEST_F(MeasurementTopicStatisticsTest, StatisticTypesTheProducerOmitsStayOutOfTheRecord)
{
  ms_node_->declare_parameter("topic_statistics.plugin", std::string("dc_measurements/TopicStatistics"));
  ms_node_->declare_parameter("topic_statistics.group_key", std::string("topic_statistics"));
  ms_node_->declare_parameter("topic_statistics.topic_output", std::string("/dc/measurement/topic_statistics"));
  ms_node_->declare_parameter("topic_statistics.init_collect", false);

  startLifecycleNode();
  waitForSubscriber("/statistics");

  ASSERT_TRUE(spinUntil([&] {
    publishMetrics({ statistics_msgs::msg::StatisticDataPoint::MAXIMUM });
    return callback_active_;
  })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["maximum"], 1.0);
  EXPECT_FALSE(data_json_.contains("average"));
  EXPECT_FALSE(data_json_.contains("minimum"));
  EXPECT_FALSE(data_json_.contains("stddev"));
  EXPECT_FALSE(data_json_.contains("sample_count"));
}

DC_MEASUREMENT_TEST_MAIN()
