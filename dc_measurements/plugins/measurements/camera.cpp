#include "dc_measurements/plugins/measurements/camera.hpp"

namespace dc_measurements
{

Camera::Camera() : dc_measurements::Measurement()
{
}

Camera::~Camera() = default;

void Camera::onConfigure()
{
  auto node = getNode();
  // Declare parameters
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".cam_name", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".cam_topic", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".draw_det_barcodes",
                                               rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".rotation_angle", rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".save_raw_img", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".save_rotated_img",
                                               rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".save_detections_img",
                                               rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".save_raw_path",
                                               rclcpp::ParameterValue("camera/raw/%Y-%m-%dT%H:%M:%S"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".save_rotated_path",
                                               rclcpp::ParameterValue("camera/rotated/%Y-%m-%dT%H:%M:%S"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".save_inspected_path",
                                               rclcpp::ParameterValue("camera/inspected/%Y-%m-%dT%H:%M:%S"));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".detection_modules",
                                               rclcpp::ParameterValue(std::vector<std::string>()));
  nav2_util::declare_parameter_if_not_declared(node, measurement_name_ + ".destinations.minio.bucket",
                                               rclcpp::ParameterValue(std::string("")));

  // Get parameters
  node->get_parameter(measurement_name_ + ".cam_name", cam_name_);
  node->get_parameter(measurement_name_ + ".draw_det_barcodes", draw_det_barcodes_);
  node->get_parameter(measurement_name_ + ".rotation_angle", rotation_angle_);
  node->get_parameter(measurement_name_ + ".cam_topic", cam_topic_);
  node->get_parameter(measurement_name_ + ".save_raw_img", save_raw_img_);
  node->get_parameter(measurement_name_ + ".save_rotated_img", save_rotated_img_);
  node->get_parameter(measurement_name_ + ".save_detections_img", save_detections_img_);
  node->get_parameter(measurement_name_ + ".save_raw_path", save_raw_path_);
  node->get_parameter(measurement_name_ + ".save_rotated_path", save_rotated_path_);
  node->get_parameter(measurement_name_ + ".save_inspected_path", save_inspected_path_);
  node->get_parameter(measurement_name_ + ".detection_modules", detection_modules_);
  node->get_parameter(measurement_name_ + ".destinations.minio.bucket", minio_bucket_);

  client_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Parameter validation and adjustments
  rotation_angle_ = rotation_angle_ % 360;
  if (rotation_angle_ % 90 != 0)
  {
    throw std::runtime_error{ "Rotation angle must be a modulo of 90" };
  }
  else if (rotation_angle_ != 0 && rotation_angle_ % 360 == 0)
  {
    RCLCPP_WARN(logger_, "Rotation angle is set to full rotation (%d). Ignored", rotation_angle_);
    rotation_angle_ = 0;
    node->set_parameter(rclcpp::Parameter(measurement_name_ + ".rotation_angle", rotation_angle_));
  }

  if (std::find(detection_modules_.begin(), detection_modules_.end(), "barcode") != detection_modules_.end())
  {
    cli_barcodes_ = node->create_client<dc_interfaces::srv::DetectBarcode>(
        "/dc/service/detect_barcodes", rmw_qos_profile_services_default, client_cb_group_);
    while (!cli_barcodes_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        throw std::runtime_error{ "Barcode client interrupted while waiting for service to appear." };
      }
      RCLCPP_INFO(logger_, "Waiting for barcode service to appear...");
    }
    RCLCPP_INFO(logger_, "Barcode detection service initialized");
  }

  // Services
  if (save_raw_img_ || save_rotated_img_ || save_detections_img_)
  {
    cli_save_img_ = node->create_client<dc_interfaces::srv::SaveImage>(
        "/dc/service/save_image", rmw_qos_profile_services_default, client_cb_group_);
    while (!cli_save_img_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        throw std::runtime_error{ "Save image client interrupted while waiting for service to appear." };
      }
      RCLCPP_INFO(logger_, "Waiting for save image service to appear...");
    }
    RCLCPP_INFO(logger_, "Save image service initialized");
  }

  if (draw_det_barcodes_)
  {
    cli_draw_image_ = node->create_client<dc_interfaces::srv::DrawImage>(
        "/dc/service/draw_image", rmw_qos_profile_services_default, client_cb_group_);
    while (!cli_draw_image_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        throw std::runtime_error{ "Draw image client interrupted while waiting for service to appear." };
      }
      RCLCPP_INFO(logger_, "Waiting for draw image service to appear...");
    }
    RCLCPP_INFO(logger_, "Draw detection barcode service initialized");
  }

  // Data subscriber
  subscription_ = node->create_subscription<sensor_msgs::msg::Image>(
      cam_topic_.c_str(), 10, std::bind(&Camera::cameraCb, this, std::placeholders::_1));
}

void Camera::cameraCb(const sensor_msgs::msg::Image& msg)
{
  last_data_ = msg;
}

void Camera::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_measurements", "camera.json");
  }
}

void Camera::saveRemoteKeys(json& data_json, const std::string& key, const std::string& relative_path,
                            const rclcpp::Time& now)
{
  for (auto it = remote_keys_.begin(); it != remote_keys_.end(); ++it)
  {
    int index = std::distance(remote_keys_.begin(), it);
    std::string new_key = *it;
    std::string new_value = std::filesystem::path(all_base_path_expanded_) / remote_prefixes_[index] / relative_path;
    new_value = new_value + ".jpg";

    // Erase leading slash
    if (new_value[0] == '/')
    {
      new_value.erase(0, 1);
    }
    // e.g data_json["remote_keys"]["minio"]["rotated"]
    data_json["remote_paths"][new_key][key] = dc_util::expand_time(new_value, now);
  }
}

std::string Camera::getLocalPath(const std::string& param_reference, const rclcpp::Time& now)
{
  std::string file_save_path = getSavePath(param_reference, now);
  std::string local_file_save_path =
      std::filesystem::path(save_local_base_path_expanded_) / all_base_path_expanded_ / (file_save_path + ".jpg");
  local_file_save_path = dc_util::expand_time(local_file_save_path, now);

  return local_file_save_path;
}

void Camera::rotateImage(cv_bridge::CvImagePtr& cv_ptr)
{
  if (rotation_angle_ == 90 || rotation_angle_ == -270)
  {
    cv::rotate(cv_ptr->image, cv_ptr->image, cv::ROTATE_90_CLOCKWISE);
  }
  else if (rotation_angle_ == 180 || rotation_angle_ == -180)
  {
    cv::rotate(cv_ptr->image, cv_ptr->image, cv::ROTATE_180);
  }
  else if (rotation_angle_ == 270 || rotation_angle_ == -90)
  {
    cv::rotate(cv_ptr->image, cv_ptr->image, cv::ROTATE_90_COUNTERCLOCKWISE);
  }
}

dc_interfaces::msg::StringStamped Camera::collect()
{
  auto node = getNode();
  auto now = node->get_clock()->now();

  json data_json;
  if (std::size(last_data_.data) != 0)
  {
    // Transform image in cv frame
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(last_data_, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
      std::string ex = e.what();
      RCLCPP_ERROR(logger_, "cv_bridge exception: %s", ex.c_str());
      throw std::runtime_error{ ex.c_str() };
    }

    // Save raw image
    if (save_raw_img_)
    {
      // Get local and relative path
      std::string absolute_path = getLocalPath("save_raw_path", now);
      std::string relative_path = getSavePath("save_raw_path", now);

      std::string absolute_path_dir = std::filesystem::path(absolute_path).parent_path().u8string();
      std::filesystem::create_directories(absolute_path_dir);
      auto status = cv::imwrite(absolute_path, cv_ptr->image);

      if (!status)
      {
        RCLCPP_ERROR(logger_, "Did not receive response from save raw image request: %d", (int)status);
      }
      else
      {
        // Save local path
        data_json["local_paths"]["raw"] = absolute_path;
        // Save all future remote path. This is then used as reference for your APIs
        saveRemoteKeys(data_json, "raw", relative_path, now);
      }
    }

    // Rotate image
    if (rotation_angle_ != 0)
    {
      rotateImage(cv_ptr);
    }

    // Save rotated image
    if (save_rotated_img_ && rotation_angle_ != 0)
    {
      // Get local and relative path
      std::string absolute_path = getLocalPath("save_rotated_path", now);
      std::string relative_path = getSavePath("save_rotated_path", now);

      std::string absolute_path_dir = std::filesystem::path(absolute_path).parent_path().u8string();
      std::filesystem::create_directories(absolute_path_dir);
      auto status = cv::imwrite(absolute_path, cv_ptr->image);

      if (!status)
      {
        RCLCPP_ERROR(logger_, "Could not save rotated image: %d", (int)status);
      }
      else
      {
        // Save local path
        data_json["local_paths"]["rotated"] = absolute_path;
        // Save all future remote path. This is then used as reference for your APIs
        saveRemoteKeys(data_json, "rotated", relative_path, now);
      }
    }

    // Start inspection
    // Barcode
    if (std::find(detection_modules_.begin(), detection_modules_.end(), "barcode") != detection_modules_.end())
    {
      auto barcode_req = std::make_shared<dc_interfaces::srv::DetectBarcode::Request>();
      barcode_req->frame = dc_util::cvToImage(cv_ptr);
      auto result_future = cli_barcodes_->async_send_request(barcode_req);
      std::future_status status = result_future.wait_for(500ms);

      if (status != std::future_status::ready)
      {
        RCLCPP_ERROR(logger_, "Did not receive response from detecting barcode in image request: %d", (int)status);
      }
      auto result_barcodes = result_future.get()->barcodes;
      if (!result_barcodes.empty())
      {
        data_json["inspected"]["barcode"] = json::array();
      }
      if (
          // Save image with inspected data
          save_detections_img_
          // Barcode(s) found
          && !result_barcodes.empty()
          // Drawing enabled
          && draw_det_barcodes_)
      {
        for (auto& barcode : result_barcodes)
        {
          auto draw_req = std::make_shared<dc_interfaces::srv::DrawImage::Request>();
          draw_req->frame = dc_util::cvToImage(cv_ptr);
          draw_req->shape = "rectangle";
          draw_req->box_top = barcode.top;
          draw_req->box_left = barcode.left;
          draw_req->box_width = barcode.width;
          draw_req->box_height = barcode.height;
          draw_req->box_thickness = 5;
          uint16_t colors[3] = { 255, 0, 0 };
          // https://answers.ros.org/question/363764/how-does-one-assign-values-to-messages-with-arrays-in-c/
          draw_req->color.assign(colors, colors + 3);
          draw_req->font_txt = cv::FONT_HERSHEY_SIMPLEX;
          draw_req->font_thickness = 2;
          draw_req->font_scale = 0.8;
          draw_req->text = std::string(barcode.type) + "-" + barcode.data;
          auto result_future_draw = cli_draw_image_->async_send_request(draw_req);
          auto status_draw = result_future_draw.wait_for(500ms);
          if (status_draw != std::future_status::ready)
          {
            RCLCPP_ERROR(logger_, "Did not receive response from drawing barcode in image request: %d", (int)status);
          }
          auto image_det = result_future_draw.get()->frame;
          cv_ptr = cv_bridge::toCvCopy(image_det, image_det.encoding);
          json barcode_json;
          barcode_json["data"] = barcode.data;
          barcode_json["type"] = barcode.type;
          barcode_json["top"] = barcode.top;
          barcode_json["left"] = barcode.left;
          barcode_json["width"] = barcode.width;
          barcode_json["height"] = barcode.height;
          data_json["inspected"]["barcode"].push_back(barcode_json);
        }
      }

      if (!detection_modules_.empty() && save_detections_img_ && dc_util::fieldInJSON(data_json, "inspected"))
      {
        // Save image with detection
        // Get local and relative path
        std::string absolute_path = getLocalPath("save_inspected_path", now);
        std::string relative_path = getSavePath("save_inspected_path", now);

        // Request to save image
        // auto status_det = dc_util::req_save_image(cli_save_img_, last_data_, absolute_path, 500ms);
        std::string absolute_path_dir = std::filesystem::path(absolute_path).parent_path().u8string();
        std::filesystem::create_directories(absolute_path_dir);
        auto status_det = cv::imwrite(absolute_path, cv_ptr->image);

        if (!status_det)
        {
          RCLCPP_ERROR(logger_, "Could not save det image request: %d", (int)status_det);
        }
        else
        {
          // Save local path
          data_json["local_paths"]["inspected"] = absolute_path;
          // Save all future remote path. This is then used as reference for your APIs
          saveRemoteKeys(data_json, "inspected", relative_path, now);
        }
      }
    }
    else
    {
      dc_interfaces::msg::StringStamped msg;
      return msg;
    }
  }
  dc_interfaces::msg::StringStamped msg;
  msg.header.stamp = now;
  msg.group_key = group_key_;
  msg.data = data_json.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Camera, dc_core::Measurement)
