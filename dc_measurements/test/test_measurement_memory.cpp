// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "measurement_test_bench.hpp"

class MeasurementMemoryTest : public MeasurementBench
{
protected:
  MeasurementMemoryTest() : MeasurementBench("memory")
  {
  }
};

TEST_F(MeasurementMemoryTest, MemoryUsedIsAPercentage)
{
  ms_node_->declare_parameter("memory.plugin", std::string("dc_measurements/Memory"));
  ms_node_->declare_parameter("memory.group_key", std::string("memory"));
  ms_node_->declare_parameter("memory.topic_output", std::string("/dc/measurement/memory"));

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_GE(data_json_["used"].get<float>(), 0.0);
  EXPECT_LE(data_json_["used"].get<float>(), 100.0);
}

DC_MEASUREMENT_TEST_MAIN()
