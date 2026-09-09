// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <sys/utsname.h>
#include <unistd.h>

#include <thread>

#include "measurement_test_bench.hpp"

class MeasurementOSTest : public MeasurementBench
{
protected:
  MeasurementOSTest() : MeasurementBench("os")
  {
  }
};

TEST_F(MeasurementOSTest, OSDataCorrect)
{
  ms_node_->declare_parameter("os.plugin", std::string("dc_measurements/OS"));
  ms_node_->declare_parameter("os.group_key", std::string("os"));
  ms_node_->declare_parameter("os.topic_output", std::string("/dc/measurement/os"));

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["cpus"].get<unsigned int>(), std::thread::hardware_concurrency());
  EXPECT_GT(data_json_["memory"].get<float>(), 0.0);
  EXPECT_LE(data_json_["memory"].get<float>(), 100.0);

  utsname result;
  uname(&result);

  EXPECT_EQ(data_json_["kernel"].get<std::string>(), std::string(result.release));
}

DC_MEASUREMENT_TEST_MAIN()
