#include <cv_bridge/cv_bridge.h>

#include <boost/algorithm/string/replace.hpp>
#include <chrono>
#include <memory>
#include <opencv2/highgui.hpp>
#include <string>

#include "dc_core/measurement.hpp"
#include "dc_interfaces/srv/detect_barcode.hpp"
#include "dc_interfaces/srv/draw_image.hpp"
#include "dc_interfaces/srv/save_image.hpp"
#include "dc_measurements/measurement.hpp"
#include "dc_measurements/measurement_server.hpp"
#include "dc_util/image_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dc_util
{

std::future_status req_save_image(const rclcpp::Client<dc_interfaces::srv::SaveImage>::SharedPtr& cli_save_img_,
                                  const sensor_msgs::msg::Image& data, const std::string& path,
                                  const std::chrono::milliseconds& timeout = std::chrono::milliseconds(500))
{
  auto save_raw_img_req = std::make_shared<dc_interfaces::srv::SaveImage::Request>();
  save_raw_img_req->frame = data;
  save_raw_img_req->path = path.c_str();
  auto result_future = cli_save_img_->async_send_request(save_raw_img_req);
  std::future_status status = result_future.wait_for(timeout);

  return status;
}

}  // namespace dc_util
