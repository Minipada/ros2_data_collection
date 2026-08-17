// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_MEASUREMENTS__CODE_POSE_HPP_
#define DC_MEASUREMENTS__CODE_POSE_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"

namespace dc_measurements
{

/**
 * @brief Pose of a square planar code (QR, DataMatrix, …) from its four image corners.
 *
 * Corners come in the order a detector reports them for the code's own orientation:
 * top-left, top-right, bottom-right, bottom-left. The code frame is centred on the code
 * with X right, Y down and Z into its printed face — the camera optical frame's own axes
 * (REP 103: X right, Y down, Z forward), so a code seen square-on has the identity
 * orientation. The returned pose is the *code's* pose in the camera optical frame.
 *
 * @param corners Detected corners, in pixels, in the raw (unrotated) image.
 * @param side_length Physical side length of the code, in metres.
 * @param intrinsics Row-major 3x3 camera matrix (sensor_msgs/CameraInfo `k`).
 * @param distortion Distortion coefficients (sensor_msgs/CameraInfo `d`); may be empty.
 * @return The pose, or nullopt if the inputs are degenerate or solvePnP fails.
 */
inline std::optional<geometry_msgs::msg::Pose> estimateSquareCodePose(const std::array<cv::Point2f, 4>& corners,
                                                                      double side_length,
                                                                      const std::array<double, 9>& intrinsics,
                                                                      const std::vector<double>& distortion)
{
  // fx/fy of zero means the CameraInfo is unset rather than merely imprecise.
  if (side_length <= 0.0 || intrinsics[0] <= 0.0 || intrinsics[4] <= 0.0)
  {
    return std::nullopt;
  }

  // Exactly the object-point layout SOLVEPNP_IPPE_SQUARE documents as its precondition.
  const auto half = static_cast<float>(side_length / 2.0);
  const std::vector<cv::Point3f> object_points = {
    { -half, half, 0.F }, { half, half, 0.F }, { half, -half, 0.F }, { -half, -half, 0.F }
  };
  const std::vector<cv::Point2f> image_points(corners.begin(), corners.end());

  cv::Mat camera_matrix(3, 3, CV_64F);
  std::copy(intrinsics.begin(), intrinsics.end(), camera_matrix.ptr<double>());
  const cv::Mat dist_coeffs = distortion.empty() ? cv::Mat::zeros(1, 5, CV_64F) : cv::Mat(distortion, true);

  cv::Vec3d rvec;
  cv::Vec3d tvec;
  try
  {
    // IPPE_SQUARE is the solver meant for exactly this case: four coplanar corners of a
    // square, given in the object-point order above.
    if (!cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec, false,
                      cv::SOLVEPNP_IPPE_SQUARE))
    {
      return std::nullopt;
    }
  }
  catch (const cv::Exception&)
  {
    return std::nullopt;
  }

  cv::Matx33d rotation;
  cv::Rodrigues(rvec, rotation);
  // That layout puts the solver's own frame Y-up with Z out of the face towards the
  // camera, which makes a square-on read a 180-degree roll. Turning it back about X (the
  // negated Y and Z columns below) is what makes square-on the identity instead.
  const tf2::Matrix3x3 basis(rotation(0, 0), -rotation(0, 1), -rotation(0, 2), rotation(1, 0), -rotation(1, 1),
                             -rotation(1, 2), rotation(2, 0), -rotation(2, 1), -rotation(2, 2));
  tf2::Quaternion orientation;
  basis.getRotation(orientation);

  geometry_msgs::msg::Pose pose;
  pose.position.x = tvec[0];
  pose.position.y = tvec[1];
  pose.position.z = tvec[2];
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

/**
 * @brief Map a point detected in a rotated image back to raw-image pixel coordinates.
 *
 * Detection runs on the rotated image, but the camera intrinsics describe the raw one.
 *
 * @param point Point in the rotated image.
 * @param rotation_angle Rotation applied to the raw image, in degrees (0/90/180/270).
 * @param raw_width Width of the raw image, in pixels.
 * @param raw_height Height of the raw image, in pixels.
 */
inline cv::Point2f unrotatePoint(const cv::Point2f& point, int rotation_angle, int raw_width, int raw_height)
{
  switch (((rotation_angle % 360) + 360) % 360)
  {
    case 90:
      return { point.y, static_cast<float>(raw_height - 1) - point.x };
    case 180:
      return { static_cast<float>(raw_width - 1) - point.x, static_cast<float>(raw_height - 1) - point.y };
    case 270:
      return { static_cast<float>(raw_width - 1) - point.y, point.x };
    default:
      return point;
  }
}

}  // namespace dc_measurements

#endif  // DC_MEASUREMENTS__CODE_POSE_HPP_
