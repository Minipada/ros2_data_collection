#ifndef DC_UTIL__IMAGE_UTILS_HPP_
#define DC_UTIL__IMAGE_UTILS_HPP_

#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <numeric>
#include <regex>
#include <vector>

#include "sensor_msgs/msg/image.hpp"

namespace dc_util
{
sensor_msgs::msg::Image cvToImage(const cv_bridge::CvImagePtr& cv_ptr)
{
  cv_bridge::CvImage img_bridge;
  img_bridge.encoding = sensor_msgs::image_encodings::BGR8;
  img_bridge.image = cv_ptr->image;
  sensor_msgs::msg::Image img_msg;
  img_bridge.toImageMsg(img_msg);

  return img_msg;
}
}  // namespace dc_util

#endif  // DC_UTIL__IMAGE_UTILS_HPP_
