// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "measurement_test_bench.hpp"

class MeasurementUptimeTest : public MeasurementBench
{
protected:
  MeasurementUptimeTest() : MeasurementBench("uptime")
  {
  }
};

TEST_F(MeasurementUptimeTest, UptimeDataCorrect)
{
  ms_node_->declare_parameter("uptime.plugin", std::string("dc_measurements/Uptime"));
  ms_node_->declare_parameter("uptime.group_key", std::string("uptime"));
  ms_node_->declare_parameter("uptime.topic_output", std::string("/dc/measurement/uptime"));

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_GT(data_json_["time"].get<unsigned int>(), 0u);
}

DC_MEASUREMENT_TEST_MAIN()
