// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "dc_util/base64.hpp"
#include "measurement_test_bench.hpp"

class MeasurementCameraTest : public MeasurementBench
{
protected:
  MeasurementCameraTest() : MeasurementBench("camera")
  {
    image_pub_ = ms_node_->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::SystemDefaultsQoS());
  }

  // BGR8, filled with a mid-grey so the encoded JPEG is never empty.
  static sensor_msgs::msg::Image makeImage(uint32_t height, uint32_t width)
  {
    sensor_msgs::msg::Image msg;
    msg.height = height;
    msg.width = width;
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = width * 3;
    msg.data.assign(static_cast<size_t>(msg.step) * height, 128);
    return msg;
  }

  static cv::Mat decodeBase64Image(const nlohmann::json& base64_str)
  {
    std::string decoded = base64_decode(base64_str.get<std::string>());
    std::vector<uchar> buf(decoded.begin(), decoded.end());
    return cv::imdecode(buf, cv::IMREAD_COLOR);
  }

  void declareCommonParams()
  {
    ms_node_->declare_parameter("camera.plugin", std::string("dc_measurements/Camera"));
    ms_node_->declare_parameter("camera.group_key", std::string("camera"));
    ms_node_->declare_parameter("camera.topic_output", std::string("/dc/measurement/camera"));
    ms_node_->declare_parameter("camera.cam_name", std::string("front"));
    ms_node_->declare_parameter("camera.cam_topic", std::string("/camera/image_raw"));
    // Avoids blocking onConfigure() on the /dc/service/draw_image service, which this test
    // does not stand up.
    ms_node_->declare_parameter("camera.draw_det_barcodes", false);
    // Collect() fires on activate() by default; disabled so no Record is published before a
    // test publishes its synthetic image.
    ms_node_->declare_parameter("camera.init_collect", false);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

TEST_F(MeasurementCameraTest, PublishesBase64EncodedRawImage)
{
  declareCommonParams();
  ms_node_->declare_parameter("camera.save_raw_base64", true);

  startLifecycleNode();
  waitForSubscriber("/camera/image_raw");

  auto image = makeImage(4, 4);
  ASSERT_TRUE(spinUntil([&] {
    image_pub_->publish(image);
    return callback_active_;
  })) << "no Record ever arrived";

  EXPECT_EQ(data_json_["camera_name"], "front");
  ASSERT_TRUE(data_json_.contains("base64"));
  ASSERT_TRUE(data_json_["base64"].contains("raw"));

  cv::Mat decoded_image = decodeBase64Image(data_json_["base64"]["raw"]);
  ASSERT_FALSE(decoded_image.empty());
  EXPECT_EQ(decoded_image.rows, 4);
  EXPECT_EQ(decoded_image.cols, 4);
}

TEST_F(MeasurementCameraTest, RotatesImageBeforeEncoding)
{
  declareCommonParams();
  ms_node_->declare_parameter("camera.rotation_angle", 90);
  ms_node_->declare_parameter("camera.save_rotated_base64", true);

  startLifecycleNode();
  waitForSubscriber("/camera/image_raw");

  // Non-square so a 90-degree rotation is observable in the output dimensions.
  auto image = makeImage(4, 8);
  ASSERT_TRUE(spinUntil([&] {
    image_pub_->publish(image);
    return callback_active_;
  })) << "no Record ever arrived";

  ASSERT_TRUE(data_json_.contains("base64"));
  ASSERT_TRUE(data_json_["base64"].contains("rotated"));

  cv::Mat decoded_image = decodeBase64Image(data_json_["base64"]["rotated"]);
  ASSERT_FALSE(decoded_image.empty());
  EXPECT_EQ(decoded_image.rows, 8);
  EXPECT_EQ(decoded_image.cols, 4);
}

TEST_F(MeasurementCameraTest, NoPublishBeforeFirstImageReceived)
{
  int polling_interval = 50;
  declareCommonParams();
  ms_node_->declare_parameter("camera.save_raw_base64", true);
  ms_node_->declare_parameter("camera.polling_interval", polling_interval);

  startLifecycleNode();

  // Poll through several collect() cycles with no image published; the plugin must not
  // publish a Record while last_data_ is still unset.
  spinFor(polling_interval * 3);

  EXPECT_FALSE(callback_active_);
}

DC_MEASUREMENT_TEST_MAIN()
