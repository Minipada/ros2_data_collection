// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "measurement_test_bench.hpp"

class MeasurementIpCameraTest : public MeasurementBench
{
protected:
  MeasurementIpCameraTest() : MeasurementBench("ip_camera")
  {
  }
};

TEST_F(MeasurementIpCameraTest, NoSegmentsYetProducesNoPublish)
{
  // Keep saves under the test's scratch dir rather than the real $HOME default.
  ms_node_->declare_parameter("save_local_base_path", std::string("/tmp/dc_measurement_ip_camera_test"));
  ms_node_->declare_parameter("ip_camera.plugin", std::string("dc_measurements/IpCamera"));
  ms_node_->declare_parameter("ip_camera.group_key", std::string("ip_camera"));
  ms_node_->declare_parameter("ip_camera.topic_output", std::string("/dc/measurement/ip_camera"));
  ms_node_->declare_parameter("ip_camera.input", std::string("/nonexistent/dc_ip_camera_test_input.mp4"));
  // The default "save_path" ("ffmpeg_%Y-%m-%dT%H:%M:%S") fails onConfigure()'s own validation
  // regex, which requires the value to *start* with '%' -- a pre-existing bug that would make
  // onConfigure() throw (and the measurement silently disable) before ever reaching the ffmpeg
  // spawn below. Overridden here so this test exercises the "ffmpeg never produces a segment"
  // path it's actually meant to cover, not that unrelated bug.
  ms_node_->declare_parameter("ip_camera.save_path", std::string("%Y-%m-%dT%H:%M:%S"));
  ms_node_->declare_parameter("ip_camera.polling_interval", 50);

  startLifecycleNode();

  // Poll through several collect() cycles; since ffmpeg never produces a segment, the storage
  // directory stays empty and collect() should never publish a Record (default StringStamped's
  // data field is left empty, which the base publish() drops).
  spinFor(300);

  EXPECT_FALSE(callback_active_);
}

DC_MEASUREMENT_TEST_MAIN()
