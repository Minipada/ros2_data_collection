#include <gtest/gtest.h>

#include <opencv2/imgcodecs.hpp>

#include "dc_interfaces/msg/string_stamped.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/base64.hpp"
#include "dc_util/json_utils.hpp"
#include "sensor_msgs/msg/image.hpp"

class MeasurementCameraTest : public ::testing::Test
{
protected:
  MeasurementCameraTest()
  {
    SetUp();
  }

  ~MeasurementCameraTest() override
  {
  }

  void SetUp() override
  {
    ms_node_ = std::make_shared<measurement_server::MeasurementServer>(rclcpp::NodeOptions(),
                                                                       std::vector<std::string>{ "camera" });
    sub_data_ = ms_node_->create_subscription<dc_interfaces::msg::StringStamped>(
        "/dc/measurement/camera", rclcpp::SystemDefaultsQoS(),
        std::bind(&MeasurementCameraTest::cameraDataCallback, this, std::placeholders::_1));
    image_pub_ = ms_node_->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::SystemDefaultsQoS());
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

  void cameraDataCallback(const dc_interfaces::msg::StringStamped& msg)
  {
    std::string data_str = msg.data.c_str();
    boost::replace_all(data_str, "'", "\"");
    RCLCPP_INFO_STREAM(ms_node_->get_logger(), "Value: " << data_str);
    data_json_ = nlohmann::json::parse(data_str);
    callback_active_ = true;
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

  void waitForImageSubscriber()
  {
    while (ms_node_->count_subscribers("/camera/image_raw") == 0)
    {
      rclcpp::spin_some(ms_node_->get_node_base_interface());
    }
  }

  std::shared_ptr<measurement_server::MeasurementServer> ms_node_;
  rclcpp::Subscription<dc_interfaces::msg::StringStamped>::SharedPtr sub_data_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  nlohmann::json data_json_;

public:
  bool callback_active_{ false };
};

TEST_F(MeasurementCameraTest, PublishesBase64EncodedRawImage)
{
  declareCommonParams();
  ms_node_->declare_parameter("camera.save_raw_base64", true);

  startLifecycleNode();
  waitForImageSubscriber();

  auto image = makeImage(4, 4);
  while (!callback_active_)
  {
    image_pub_->publish(image);
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

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
  waitForImageSubscriber();

  // Non-square so a 90-degree rotation is observable in the output dimensions.
  auto image = makeImage(4, 8);
  while (!callback_active_)
  {
    image_pub_->publish(image);
    rclcpp::spin_some(ms_node_->get_node_base_interface());
  }

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

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  // Poll through several collect() cycles with no image published; the plugin must not
  // publish a Record while last_data_ is still unset.
  while ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time)).count() <
         polling_interval * 3)
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
