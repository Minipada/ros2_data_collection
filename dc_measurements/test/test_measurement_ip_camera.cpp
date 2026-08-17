// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <gtest/gtest.h>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/json_utils.hpp"

// IpCamera::onConfigure() spawns a detached background ffmpeg process reading from "input" and
// segmenting to disk; collect() then picks up completed segment files from that directory. A
// real happy-path test would need a live video source and a working ffmpeg pipeline, neither of
// which this sandbox can provide deterministically. This suite instead points "input" at a path
// that doesn't exist, so ffmpeg fails immediately (no hang, no real video needed) and no segment
// ever appears -- covering that the plugin doesn't crash or publish garbage while idle.
class MeasurementIpCameraTest : public ::testing::Test
{
protected:
  MeasurementIpCameraTest()
  {
    SetUp();
  }

  ~MeasurementIpCameraTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "ip_camera" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/ip_camera", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementIpCameraTest::ipCameraDataCallback, this, std::placeholders::_1));
  }

  void TearDown() override
  {
    ms_node_->deactivate();
    ms_node_->cleanup();
  }

  void startLifecycleNode()
  {
    ms_node_->configure();
    ms_node_->activate();
  }

  void ipCameraDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  nlohmann::json data_json_;

public:
  bool callback_active_{ false };
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

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  // Poll through several collect() cycles; since ffmpeg never produces a segment, the storage
  // directory stays empty and collect() should never publish a Record (default StringStamped's
  // data field is left empty, which the base publish() drops).
  while ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time)).count() <
         300)
  {
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

  EXPECT_FALSE(callback_active_);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
