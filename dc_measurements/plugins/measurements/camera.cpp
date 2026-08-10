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
  cam_name_ = dc_util::get_str_type_param(node, measurement_name_, "cam_name");
  cam_topic_ = dc_util::get_str_type_param(node, measurement_name_, "cam_topic");
  draw_det_barcodes_ = dc_util::get_bool_type_param(node, measurement_name_, "draw_det_barcodes", true);
  rotation_angle_ = dc_util::get_int_type_param(node, measurement_name_, "rotation_angle", 0);
  save_raw_img_ = dc_util::get_bool_type_param(node, measurement_name_, "save_raw_img", false);
  save_rotated_img_ = dc_util::get_bool_type_param(node, measurement_name_, "save_rotated_img", false);
  save_detections_img_ = dc_util::get_bool_type_param(node, measurement_name_, "save_detections_img", true);
  save_raw_path_ =
      dc_util::get_str_type_param(node, measurement_name_, "save_raw_path", "camera/raw/%Y-%m-%dT%H:%M:%S");
  save_raw_base64_ = dc_util::get_bool_type_param(node, measurement_name_, "save_raw_base64", false);
  save_rotated_path_ =
      dc_util::get_str_type_param(node, measurement_name_, "save_rotated_path", "camera/rotated/%Y-%m-%dT%H:%M:%S");
  save_rotated_base64_ = dc_util::get_bool_type_param(node, measurement_name_, "save_rotated_base64", false);
  save_inspected_path_ =
      dc_util::get_str_type_param(node, measurement_name_, "save_inspected_path", "camera/inspected/%Y-%m-%dT%H:%M:%S");
  save_inspected_base64_ = dc_util::get_bool_type_param(node, measurement_name_, "save_inspected_base64", false);
  detection_modules_ =
      dc_util::get_str_array_type_param(node, measurement_name_, "detection_modules", std::vector<std::string>());
  // FIXME Not used, wrong parameter too, should be without measurement_name_
  minio_bucket_ = dc_util::get_str_type_param(node, measurement_name_, "destinations.minio.bucket", "");

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

  if (draw_det_barcodes_)
  {
    cli_draw_image_ = node->create_client<dc_interfaces::srv::DrawImage>("/dc/service/draw_image",
                                                                         rclcpp::ServicesQoS(), client_cb_group_);
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
      cam_topic_, 10, std::bind(&Camera::cameraCb, this, std::placeholders::_1));
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

    if (save_raw_base64_)
    {
      std::vector<uchar> buf;
      cv::imencode(".jpg", cv_ptr->image, buf);
      auto* enc_msg = reinterpret_cast<unsigned char*>(buf.data());
      data_json["base64"]["raw"] = base64_encode(enc_msg, buf.size());
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

    if (save_rotated_base64_)
    {
      std::vector<uchar> buf;
      cv::imencode(".jpg", cv_ptr->image, buf);
      auto* enc_msg = reinterpret_cast<unsigned char*>(buf.data());
      data_json["base64"]["rotated"] = base64_encode(enc_msg, buf.size());
    }

    // Start inspection
    // Barcode
    if (std::find(detection_modules_.begin(), detection_modules_.end(), "barcode") != detection_modules_.end())
    {
      // In-process detection via ZXing-C++ (#123): replaces the old per-frame round trip to
      // the Python /dc/service/detect_barcodes service (pyzbar/zbar).
      ZXing::ImageView image_view(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, ZXing::ImageFormat::BGR,
                                  static_cast<int>(cv_ptr->image.step));
      auto results = ZXing::ReadBarcodes(image_view);
      std::vector<ZXing::Result> result_barcodes;
      for (auto& result : results)
      {
        if (result.isValid())
        {
          result_barcodes.push_back(result);
        }
      }

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
          auto bbox = ZXing::BoundingBox(barcode.position());
          uint16_t top = static_cast<uint16_t>(std::max(0, bbox.topLeft().y));
          uint16_t left = static_cast<uint16_t>(std::max(0, bbox.topLeft().x));
          uint16_t width = static_cast<uint16_t>(bbox.bottomRight().x - bbox.topLeft().x);
          uint16_t height = static_cast<uint16_t>(bbox.bottomRight().y - bbox.topLeft().y);
          std::string type = ZXing::ToString(barcode.format());
          std::string data = barcode.text();

          auto draw_req = std::make_shared<dc_interfaces::srv::DrawImage::Request>();
          draw_req->frame = dc_util::cvToImage(cv_ptr);
          draw_req->shape = "rectangle";
          draw_req->box_top = top;
          draw_req->box_left = left;
          draw_req->box_width = width;
          draw_req->box_height = height;
          draw_req->box_thickness = 5;
          uint16_t colors[3] = { 255, 0, 0 };
          // https://answers.ros.org/question/363764/how-does-one-assign-values-to-messages-with-arrays-in-c/
          draw_req->color.assign(colors, colors + 3);
          draw_req->font_txt = cv::FONT_HERSHEY_SIMPLEX;
          draw_req->font_thickness = 2;
          draw_req->font_scale = 0.8;
          draw_req->text = type + "-" + data;
          auto result_future_draw = cli_draw_image_->async_send_request(draw_req);
          auto status_draw = result_future_draw.wait_for(500ms);
          if (status_draw != std::future_status::ready)
          {
            RCLCPP_ERROR(logger_, "Did not receive response from drawing barcode in image request: %d",
                         (int)status_draw);
          }
          auto image_det = result_future_draw.get()->frame;
          cv_ptr = cv_bridge::toCvCopy(image_det, image_det.encoding);
          json barcode_json;
          barcode_json["data"] = data;
          barcode_json["type"] = type;
          barcode_json["top"] = top;
          barcode_json["left"] = left;
          barcode_json["width"] = width;
          barcode_json["height"] = height;
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

      if (save_inspected_base64_)
      {
        std::vector<uchar> buf;
        cv::imencode(".jpg", cv_ptr->image, buf);
        auto* enc_msg = reinterpret_cast<unsigned char*>(buf.data());
        data_json["base64"]["inspected"] = base64_encode(enc_msg, buf.size());
      }
    }
  }
  dc_interfaces::msg::StringStamped msg;
  if (data_json.empty())
  {
    return msg;
  }
  msg.header.stamp = now;
  msg.group_key = group_key_;
  data_json["camera_name"] = cam_name_;
  msg.data = data_json.dump(-1, ' ', true);
  return msg;
}

}  // namespace dc_measurements

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_measurements::Camera, dc_core::Measurement)
