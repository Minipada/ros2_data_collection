// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "measurement_test_bench.hpp"

class MeasurementMapTest : public MeasurementBench
{
protected:
  MeasurementMapTest() : MeasurementBench("map")
  {
  }
};

TEST_F(MeasurementMapTest, NoMapDataProducesNoPublish)
{
  // Keep saves under the test's scratch dir rather than the real $HOME default, and keep
  // map_saver_cli's own wait for /map short in case it is actually installed.
  ms_node_->declare_parameter("save_local_base_path", std::string("/tmp/dc_measurement_map_test"));
  ms_node_->declare_parameter("map.plugin", std::string("dc_measurements/Map"));
  ms_node_->declare_parameter("map.group_key", std::string("map"));
  ms_node_->declare_parameter("map.topic_output", std::string("/dc/measurement/map"));
  ms_node_->declare_parameter("map.topic", std::string("/test/map"));
  ms_node_->declare_parameter("map.save_map_timeout", 1.0);

  startLifecycleNode();

  // With no /map data ever published, every collect() cycle returns an empty StringStamped, and
  // Measurement::publish() (measurement.hpp) drops empty messages outright (logs a WARN, never
  // calls data_pub_->publish()) rather than publishing "{}" -- so the callback can never fire.
  // Each real collect() cycle here costs ~1.3s (the map_saver_cli subprocess spawn + its 1s
  // save_map_timeout), so the poll window is longer than the other plugins' 300ms to actually
  // exercise a couple of cycles rather than trivially passing before the first one completes.
  spinFor(4000);

  EXPECT_FALSE(callback_active_);
}

DC_MEASUREMENT_TEST_MAIN()
