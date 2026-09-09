// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "measurement_test_bench.hpp"

class MeasurementStorageTest : public MeasurementBench
{
protected:
  MeasurementStorageTest() : MeasurementBench("storage")
  {
  }
};

TEST_F(MeasurementStorageTest, PublishesFreeAndCapacityForConfiguredPath)
{
  ms_node_->declare_parameter("storage.plugin", std::string("dc_measurements/Storage"));
  ms_node_->declare_parameter("storage.group_key", std::string("storage"));
  ms_node_->declare_parameter("storage.topic_output", std::string("/dc/measurement/storage"));
  ms_node_->declare_parameter("storage.path", std::string("/tmp"));

  startLifecycleNode();

  ASSERT_TRUE(spinUntil([this] { return callback_active_; })) << "no Record ever arrived";

  ASSERT_TRUE(data_json_.contains("capacity"));
  ASSERT_TRUE(data_json_.contains("free"));
  ASSERT_TRUE(data_json_.contains("free_percent"));

  auto capacity = data_json_["capacity"].get<int64_t>();
  auto free = data_json_["free"].get<int64_t>();
  auto free_percent = data_json_["free_percent"].get<double>();

  EXPECT_GT(capacity, 0);
  EXPECT_GE(free, 0);
  EXPECT_LE(free, capacity);
  EXPECT_GE(free_percent, 0.0);
  EXPECT_LE(free_percent, 100.0);
}

// A missing mandatory parameter (e.g. "storage.path", which has no default) is not something
// this suite can safely test: dc_util::get_param_or_fatal() calls exit(-1) directly rather than
// throwing a catchable exception, so it takes the whole test binary down -- including whatever
// other TEST_F cases share the process -- rather than just failing one assertion. Left untested
// here; a real regression test for that path would need a gtest death test in a suite of its own.

DC_MEASUREMENT_TEST_MAIN()
