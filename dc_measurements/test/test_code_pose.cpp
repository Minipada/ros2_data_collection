#include <ZXing/BitMatrix.h>
#include <ZXing/MultiFormatWriter.h>
#include <ZXing/ReadBarcode.h>
#include <gtest/gtest.h>

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "dc_measurements/code_pose.hpp"

namespace
{

// A plausible 640x480 pinhole camera, no distortion.
constexpr double kFx = 525.0;
constexpr double kFy = 525.0;
constexpr double kCx = 320.0;
constexpr double kCy = 240.0;
constexpr double kSide = 0.2;

const std::array<double, 9> kIntrinsics = { kFx, 0.0, kCx, 0.0, kFy, kCy, 0.0, 0.0, 1.0 };

cv::Mat cameraMatrix(const std::array<double, 9>& intrinsics)
{
  cv::Mat camera_matrix(3, 3, CV_64F);
  std::copy(intrinsics.begin(), intrinsics.end(), camera_matrix.ptr<double>());
  return camera_matrix;
}

// The four corners of a `side`-metre code, in solvePnP's object-point layout, ordered
// top-left, top-right, bottom-right, bottom-left as a detector reports them.
std::vector<cv::Point3f> codeCorners(double side)
{
  const auto half = static_cast<float>(side / 2.0);
  return { { -half, half, 0.F }, { half, half, 0.F }, { half, -half, 0.F }, { -half, -half, 0.F } };
}

// Every test states its ground truth in the frame estimateSquareCodePose() reports, where
// the identity is a code seen square-on. Projecting it needs the same frame turned 180
// degrees about X, which is the one those object points live in.
cv::Vec3d projectionRvec(const cv::Vec3d& code_rvec)
{
  cv::Matx33d rotation;
  cv::Rodrigues(code_rvec, rotation);
  cv::Vec3d rvec;
  cv::Rodrigues(rotation * cv::Matx33d(1, 0, 0, 0, -1, 0, 0, 0, -1), rvec);
  return rvec;
}

// Projects a code of side `kSide` at a known pose into image corners, so a test can assert
// that estimateSquareCodePose() recovers the pose it started from.
std::array<cv::Point2f, 4> projectCorners(const cv::Vec3d& code_rvec, const cv::Vec3d& tvec)
{
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(codeCorners(kSide), projectionRvec(code_rvec), tvec, cameraMatrix(kIntrinsics),
                    cv::Mat::zeros(1, 5, CV_64F), image_points);
  return { image_points[0], image_points[1], image_points[2], image_points[3] };
}

}  // namespace

TEST(CodePoseTest, RecoversTranslationOfAStraightOnCode)
{
  const cv::Vec3d tvec(0.1, -0.05, 1.5);
  auto pose = dc_measurements::estimateSquareCodePose(projectCorners(cv::Vec3d(0, 0, 0), tvec), kSide, kIntrinsics, {});

  ASSERT_TRUE(pose.has_value());
  // Tolerances throughout are set by the float corner points, not by the solver.
  EXPECT_NEAR(pose->position.x, tvec[0], 1e-4);
  EXPECT_NEAR(pose->position.y, tvec[1], 1e-4);
  EXPECT_NEAR(pose->position.z, tvec[2], 1e-4);
  // Square-on to the camera is the identity rotation.
  EXPECT_NEAR(std::abs(pose->orientation.w), 1.0, 1e-4);
}

TEST(CodePoseTest, RecoversRotationOfATiltedCode)
{
  // 25 degrees about the camera's Y (yaw of the code away from the lens axis).
  const cv::Vec3d rvec(0.0, 25.0 * CV_PI / 180.0, 0.0);
  const cv::Vec3d tvec(0.0, 0.0, 2.0);
  auto pose = dc_measurements::estimateSquareCodePose(projectCorners(rvec, tvec), kSide, kIntrinsics, {});

  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->position.z, tvec[2], 1e-4);

  tf2::Quaternion q(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // 0.01 rad is about 0.6 degrees, the floor set by float corner points on a code only
  // ~50 px wide at this distance. A convention error is tens of degrees, not tenths.
  EXPECT_NEAR(roll, 0.0, 0.01);
  EXPECT_NEAR(pitch, 25.0 * CV_PI / 180.0, 0.01);
  EXPECT_NEAR(yaw, 0.0, 0.01);
}

TEST(CodePoseTest, RejectsUnusableInputs)
{
  const auto corners = projectCorners(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 1.0));

  // No physical size configured.
  EXPECT_FALSE(dc_measurements::estimateSquareCodePose(corners, 0.0, kIntrinsics, {}).has_value());
  // An all-zero CameraInfo, i.e. none published yet.
  EXPECT_FALSE(dc_measurements::estimateSquareCodePose(corners, kSide, { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, {}).has_value());
}

TEST(CodePoseTest, UnrotatesPointsBackToRawImageCoordinates)
{
  // A 640x480 raw image rotated clockwise is 480x640; its rotated (0, 0) is the raw
  // image's bottom-left corner.
  EXPECT_EQ(dc_measurements::unrotatePoint({ 0.F, 0.F }, 90, 640, 480), cv::Point2f(0.F, 479.F));
  EXPECT_EQ(dc_measurements::unrotatePoint({ 0.F, 0.F }, 180, 640, 480), cv::Point2f(639.F, 479.F));
  EXPECT_EQ(dc_measurements::unrotatePoint({ 0.F, 0.F }, 270, 640, 480), cv::Point2f(639.F, 0.F));
  EXPECT_EQ(dc_measurements::unrotatePoint({ 12.F, 34.F }, 0, 640, 480), cv::Point2f(12.F, 34.F));
  // Negative angles name the same rotations.
  EXPECT_EQ(dc_measurements::unrotatePoint({ 5.F, 7.F }, -90, 640, 480),
            dc_measurements::unrotatePoint({ 5.F, 7.F }, 270, 640, 480));
}

// A code detected in a rotated image must yield the same pose as the raw image would, once
// its corners are mapped back — otherwise the intrinsics silently describe the wrong image.
TEST(CodePoseTest, PoseIsUnchangedByImageRotation)
{
  const cv::Vec3d tvec(0.1, -0.05, 1.5);
  const auto raw_corners = projectCorners(cv::Vec3d(0, 0, 0), tvec);
  const int raw_width = 640;
  const int raw_height = 480;

  // Rotate the corners clockwise into a 480x640 image, the inverse of unrotatePoint().
  std::array<cv::Point2f, 4> rotated_corners{};
  for (size_t i = 0; i < raw_corners.size(); ++i)
  {
    rotated_corners[i] = { static_cast<float>(raw_height - 1) - raw_corners[i].y, raw_corners[i].x };
  }

  std::array<cv::Point2f, 4> recovered{};
  for (size_t i = 0; i < rotated_corners.size(); ++i)
  {
    recovered[i] = dc_measurements::unrotatePoint(rotated_corners[i], 90, raw_width, raw_height);
  }

  auto pose = dc_measurements::estimateSquareCodePose(recovered, kSide, kIntrinsics, {});
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->position.x, tvec[0], 1e-4);
  EXPECT_NEAR(pose->position.y, tvec[1], 1e-4);
  EXPECT_NEAR(pose->position.z, tvec[2], 1e-4);
}

namespace
{

// The demo camera: dc_simulation's rgbd_camera, 1280x720 at a 1.047 rad horizontal FOV.
constexpr int kDemoWidth = 1280;
constexpr int kDemoHeight = 720;
constexpr double kDemoFocal = (kDemoWidth / 2.0) / 0.5773502691896257;  // tan(1.047 / 2)
const std::array<double, 9> kDemoIntrinsics = { kDemoFocal, 0.0,        kDemoWidth / 2.0,
                                                0.0,        kDemoFocal, kDemoHeight / 2.0,
                                                0.0,        0.0,        1.0 };

// A QR code as a white image with a quiet zone; the module area starts `quiet_modules *
// module_px` in from each edge.
cv::Mat renderQrCode(const std::string& text, int module_px, int quiet_modules)
{
  auto matrix = ZXing::MultiFormatWriter(ZXing::BarcodeFormat::QRCode).setMargin(0).encode(text, 0, 0);
  cv::Mat modules(matrix.height(), matrix.width(), CV_8UC1);
  for (int y = 0; y < matrix.height(); ++y)
  {
    for (int x = 0; x < matrix.width(); ++x)
    {
      modules.at<uint8_t>(y, x) = matrix.get(x, y) ? 0 : 255;
    }
  }
  cv::Mat scaled;
  cv::resize(modules, scaled, cv::Size(), module_px, module_px, cv::INTER_NEAREST);
  cv::Mat bordered;
  const int border = quiet_modules * module_px;
  cv::copyMakeBorder(scaled, bordered, border, border, border, border, cv::BORDER_CONSTANT, 255);
  return bordered;
}

// Renders `code` as it would appear to the demo camera with the code at `rvec`/`tvec`.
cv::Mat renderView(const cv::Mat& code, double side, int border, const cv::Vec3d& rvec, const cv::Vec3d& tvec)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(codeCorners(side), projectionRvec(rvec), tvec, cameraMatrix(kDemoIntrinsics),
                    cv::Mat::zeros(1, 5, CV_64F), projected);

  const auto right = static_cast<float>(code.cols - border);
  const auto bottom = static_cast<float>(code.rows - border);
  const auto edge = static_cast<float>(border);
  const std::vector<cv::Point2f> source = { { edge, edge }, { right, edge }, { right, bottom }, { edge, bottom } };

  cv::Mat code_bgr;
  cv::cvtColor(code, code_bgr, cv::COLOR_GRAY2BGR);
  cv::Mat view(kDemoHeight, kDemoWidth, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::warpPerspective(code_bgr, view, cv::getPerspectiveTransform(source, projected), view.size(), cv::INTER_LINEAR,
                      cv::BORDER_TRANSPARENT);
  return view;
}

}  // namespace

// End to end over the real detector: render a QR code at a pose the demo camera could see
// it from, let ZXing find its corners, and check the pose comes back. This is the check
// that would catch a corner ordering or axis convention that only looks right on
// synthetic corner points.
TEST(CodePoseTest, RecoversAPoseFromARenderedQrCode)
{
  // dc_simulation's printed code, near enough: 0.325 m at a station's stopping distance.
  constexpr double kDemoSide = 0.325;
  constexpr int kBorderModules = 4;
  constexpr int kModulePx = 8;
  const cv::Vec3d rvec(0.0, 15.0 * CV_PI / 180.0, 0.0);
  const cv::Vec3d tvec(0.05, -0.02, 1.2);

  const cv::Mat code = renderQrCode("0001", kModulePx, kBorderModules);
  const cv::Mat view = renderView(code, kDemoSide, kBorderModules * kModulePx, rvec, tvec);

  ZXing::ImageView image_view(view.data, view.cols, view.rows, ZXing::ImageFormat::BGR, static_cast<int>(view.step));
  auto results = ZXing::ReadBarcodes(image_view);
  ASSERT_EQ(results.size(), 1U) << "the rendered code did not decode";
  ASSERT_EQ(results[0].text(), "0001");

  const auto& p = results[0].position();
  const std::array<cv::Point2f, 4> corners = {
    cv::Point2f(static_cast<float>(p.topLeft().x), static_cast<float>(p.topLeft().y)),
    cv::Point2f(static_cast<float>(p.topRight().x), static_cast<float>(p.topRight().y)),
    cv::Point2f(static_cast<float>(p.bottomRight().x), static_cast<float>(p.bottomRight().y)),
    cv::Point2f(static_cast<float>(p.bottomLeft().x), static_cast<float>(p.bottomLeft().y))
  };

  auto pose = dc_measurements::estimateSquareCodePose(corners, kDemoSide, kDemoIntrinsics, {});
  ASSERT_TRUE(pose.has_value());

  const double range = std::hypot(std::hypot(pose->position.x, pose->position.y), pose->position.z);
  const double truth = cv::norm(tvec);
  std::cerr << "  rendered-code pose: (" << pose->position.x << ", " << pose->position.y << ", " << pose->position.z
            << "), range " << range << " m against " << truth << " m\n";

  // Loose on purpose: a detector reports corners to within about a module, which is a
  // few percent of a code this size, so this bounds a systematic bias rather than noise.
  EXPECT_NEAR(range, truth, 0.05 * truth);
  EXPECT_NEAR(pose->position.x, tvec[0], 0.05);
  EXPECT_NEAR(pose->position.y, tvec[1], 0.05);

  tf2::Quaternion q(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  EXPECT_NEAR(pitch, rvec[1], 5.0 * CV_PI / 180.0);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
