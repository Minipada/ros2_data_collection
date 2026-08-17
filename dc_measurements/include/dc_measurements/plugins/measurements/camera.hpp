
#ifndef DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__CAMERA_HPP_
#define DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__CAMERA_HPP_

#include <ZXing/ReadBarcode.h>

#include <boost/algorithm/string/replace.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include <opencv2/highgui.hpp>

#include "dc_core/measurement.hpp"
#include "dc_interfaces/srv/draw_image.hpp"
#include "dc_measurements/code_pose.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_util/base64.hpp"
#include "dc_util/image_utils.hpp"
#include "dc_util/json_utils.hpp"
#include "dc_util/node_utils.hpp"
#include "dc_util/service_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dc_measurements
{

class Camera : public dc_measurements::Measurement
{
public:
  Camera();
  ~Camera() override;
  dc_interfaces::msg::StringStamped collect() override;

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  sensor_msgs::msg::Image last_data_;
  void cameraCb(const sensor_msgs::msg::Image& msg);
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  sensor_msgs::msg::CameraInfo last_camera_info_;
  bool camera_info_received_{ false };
  void cameraInfoCb(const sensor_msgs::msg::CameraInfo& msg);

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;
  void setValidationSchema() override;
  void rotateImage(cv_bridge::CvImagePtr& cv_ptr);
  std::string getLocalPath(const std::string& param_reference, const rclcpp::Time& now);
  void saveRemoteKeys(json& data_json, const std::string& key, const std::string& relative_path,
                      const rclcpp::Time& now);
  void addCodePose(json& barcode_json, const ZXing::Position& position, const cv::Size& raw_size);

  std::string cam_name_;
  bool draw_det_barcodes_;
  int rotation_angle_;
  std::string cam_topic_;
  bool save_raw_img_;
  bool save_rotated_img_;
  bool save_detections_img_;
  bool save_raw_base64_;
  bool save_rotated_base64_;
  bool save_inspected_base64_;
  std::string tmp_base_storage_dir_;
  std::string save_raw_path_;
  std::string save_rotated_path_;
  std::string save_inspected_path_;
  std::string minio_bucket_;
  std::vector<std::string> detection_modules_;
  bool estimate_pose_;
  double code_size_;
  std::string camera_info_topic_;
  std::string pose_frame_;
  double transform_timeout_;
  rclcpp::Client<dc_interfaces::srv::DrawImage>::SharedPtr cli_draw_image_;

  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
};

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__PLUGINS__MEASUREMENTS__CAMERA_HPP_
