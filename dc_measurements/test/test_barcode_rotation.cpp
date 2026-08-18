// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include <ZXing/BitMatrix.h>
#include <ZXing/MultiFormatWriter.h>
#include <ZXing/ReadBarcode.h>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <string>

namespace
{

// dc_simulation's qrcode_0001.png, to the pixel: a 210 px module area at (45, 100) in a
// 290x365 texture, the rest of the height taken by the printed label. #297 was measured by
// rotating that texture inside its own bounds, so these proportions are the ones that
// reproduce it.
constexpr int kTextureWidth = 290;
constexpr int kTextureHeight = 365;
constexpr int kModuleOriginX = 45;
constexpr int kModuleOriginY = 100;
constexpr int kModuleSide = 210;

// The module area's corners, which are what a rotation has to keep inside the frame -- the
// three finder patterns sit in them.
const std::array<cv::Point2f, 4> kModuleCorners = {
  cv::Point2f(kModuleOriginX, kModuleOriginY), cv::Point2f(kModuleOriginX + kModuleSide, kModuleOriginY),
  cv::Point2f(kModuleOriginX + kModuleSide, kModuleOriginY + kModuleSide),
  cv::Point2f(kModuleOriginX, kModuleOriginY + kModuleSide)
};

// The demo texture: a QR code with its printed label above it, on white.
cv::Mat renderTexture(const std::string& text)
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
  cv::resize(modules, scaled, cv::Size(kModuleSide, kModuleSide), 0, 0, cv::INTER_NEAREST);
  cv::Mat scaled_bgr;
  cv::cvtColor(scaled, scaled_bgr, cv::COLOR_GRAY2BGR);

  cv::Mat texture(kTextureHeight, kTextureWidth, CV_8UC3, cv::Scalar(255, 255, 255));
  scaled_bgr.copyTo(texture(cv::Rect(kModuleOriginX, kModuleOriginY, kModuleSide, kModuleSide)));
  cv::putText(texture, text, cv::Point(70, 70), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 0, 0), 5);
  return texture;
}

cv::Mat rotationAbout(const cv::Point2f& centre, double degrees)
{
  return cv::getRotationMatrix2D(centre, degrees, 1.0);
}

cv::Mat rotate(const cv::Mat& image, const cv::Mat& transform, const cv::Size& frame)
{
  cv::Mat rotated;
  cv::warpAffine(image, rotated, transform, frame, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
  return rotated;
}

// How many of the module area's corners the frame cuts off once rotated.
int cornersOutsideFrame(const cv::Mat& transform, const cv::Size& frame)
{
  int outside = 0;
  for (const auto& corner : kModuleCorners)
  {
    const double x =
        transform.at<double>(0, 0) * corner.x + transform.at<double>(0, 1) * corner.y + transform.at<double>(0, 2);
    const double y =
        transform.at<double>(1, 0) * corner.x + transform.at<double>(1, 1) * corner.y + transform.at<double>(1, 2);
    if (x < 0.0 || x >= frame.width || y < 0.0 || y >= frame.height)
    {
      ++outside;
    }
  }
  return outside;
}

// Reads `image` exactly the way the camera plugin does.
std::string decode(const cv::Mat& image)
{
  ZXing::ImageView view(image.data, image.cols, image.rows, ZXing::ImageFormat::BGR, static_cast<int>(image.step));
  for (const auto& result : ZXing::ReadBarcodes(view))
  {
    if (result.isValid())
    {
      return result.text();
    }
  }
  return "";
}

}  // namespace

// #297 reported that ZXing-C++ cannot decode QR codes rotated 30-45 degrees in-plane. It
// can, at any angle: the whole code just has to stay inside the frame. This is the
// assertion that report's claim would fail.
TEST(BarcodeRotationTest, DecodesACodeAtEveryInPlaneRotation)
{
  const cv::Mat texture = renderTexture("0001");
  // A square frame wide enough for the texture's diagonal, so no angle can push the code
  // out of it -- a code a camera keeps fully in view, in other words.
  const auto side = static_cast<int>(std::ceil(std::hypot(texture.cols, texture.rows)));
  const cv::Size frame(side, side);
  const cv::Point2f centre(side / 2.0F, side / 2.0F);

  cv::Mat framed(frame, CV_8UC3, cv::Scalar(255, 255, 255));
  texture.copyTo(framed(cv::Rect((side - texture.cols) / 2, (side - texture.rows) / 2, texture.cols, texture.rows)));

  for (int degrees = 0; degrees <= 90; degrees += 5)
  {
    const cv::Mat rotated = rotate(framed, rotationAbout(centre, degrees), frame);
    EXPECT_EQ(decode(rotated), "0001") << "missed the code at " << degrees << " degrees";
  }
}

// The real failure mode behind #297, and the one worth warning users about: rotating the
// demo texture inside its own bounds swings a corner of the code out of frame between 30
// and 75 degrees, because the label offsets the code from the centre it turns about. A
// code the frame cuts a finder pattern off is unreadable at any angle, by any decoder --
// nothing about those angles is special to ZXing-C++.
TEST(BarcodeRotationTest, LosesACodeTheFrameClips)
{
  const cv::Mat texture = renderTexture("0001");
  const cv::Size frame(kTextureWidth, kTextureHeight);
  const cv::Point2f centre(kTextureWidth / 2.0F, kTextureHeight / 2.0F);

  for (int degrees = 0; degrees <= 90; degrees += 15)
  {
    const cv::Mat transform = rotationAbout(centre, degrees);
    const int outside = cornersOutsideFrame(transform, frame);
    const std::string text = decode(rotate(texture, transform, frame));

    std::cerr << "  " << degrees << " degrees inside the texture's own " << kTextureWidth << "x" << kTextureHeight
              << " bounds: " << outside << "/4 code corners clipped away, decoded \""
              << (text.empty() ? "<nothing>" : text) << "\"\n";

    // The angles #297 attributed to the library are exactly the angles that clip.
    EXPECT_EQ(outside > 0, degrees >= 30 && degrees <= 75) << "at " << degrees << " degrees";
    EXPECT_EQ(text, outside > 0 ? "" : "0001") << "at " << degrees << " degrees";
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
