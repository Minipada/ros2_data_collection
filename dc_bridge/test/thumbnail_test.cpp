// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// The pure halves of optional thumbnail generation (#256): which Files get a preview,
// where it lands remotely, and where its scratch file lands locally. The Uploader's own
// thumbnail behaviour (ordering against the primary upload, failure isolation, replay
// idempotence) is covered in uploader_test.cpp against a fake generator.
#include "dc_bridge/uploader/thumbnail.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <utility>

#include "dc_bridge/uploader/content_type.hpp"

using namespace dc_bridge;
using namespace dc_bridge::uploader;

TEST(Thumbnail, DerivesPreviewsFromImagesAndVideosOnly)
{
  // The content types content_type::sniff actually produces for DC's own Measurements.
  EXPECT_TRUE(thumbnailable("image/jpeg"));                // camera
  EXPECT_TRUE(thumbnailable("image/png"));                 // map PNG
  EXPECT_TRUE(thumbnailable("image/x-portable-graymap"));  // map PGM
  EXPECT_TRUE(thumbnailable("video/mp4"));
  EXPECT_TRUE(thumbnailable("video/x-matroska"));

  // Everything else is left alone rather than handed to a decoder that would just fail.
  EXPECT_FALSE(thumbnailable("text/plain; charset=utf-8"));  // map YAML
  EXPECT_FALSE(thumbnailable("application/pdf"));
  EXPECT_FALSE(thumbnailable("application/octet-stream"));
  EXPECT_FALSE(thumbnailable(""));
}

TEST(Thumbnail, ContentTypeImageAndVideoPredicatesAgree)
{
  EXPECT_TRUE(content_type::is_image("image/gif"));
  EXPECT_FALSE(content_type::is_image("video/mp4"));
  EXPECT_FALSE(content_type::is_video("image/gif"));
  // A type is never both, so `thumbnailable` can't double-count.
  for (const char* t : { "image/jpeg", "video/mp4", "text/plain" })
  {
    EXPECT_FALSE(content_type::is_image(t) && content_type::is_video(t)) << t;
  }
}

TEST(Thumbnail, RemotePathIsTheOriginalPlusTheDocumentedSuffix)
{
  // #256 names this shape explicitly (`<path>.thumb.jpg`), and the demo dashboard's
  // gallery derives a preview's key from the original's rather than querying for it —
  // so this is a consumer-visible contract, not an internal detail.
  EXPECT_EQ(thumbnail_remote_path("cam/2026/img.jpg"), "cam/2026/img.jpg.thumb.jpg");
  EXPECT_EQ(thumbnail_remote_path("robot/map.pgm"), "robot/map.pgm.thumb.jpg");
  // Suffixed, never substituted: two Files differing only in extension keep distinct
  // previews.
  EXPECT_NE(thumbnail_remote_path("a/x.jpg"), thumbnail_remote_path("a/x.png"));
}

TEST(Thumbnail, ScratchPathsAreDistinctPerStorageAndKey)
{
  const std::string dir = "/var/lib/dc/uploader/thumbs";
  std::set<std::string> paths{
    thumbnail_scratch_path(dir, "minio", "cam/img.jpg"),
    thumbnail_scratch_path(dir, "minio", "cam/other.jpg"),
    // The same File on two Destinations is generated twice (each store is verified
    // independently) and must not race on one scratch path.
    thumbnail_scratch_path(dir, "s3_archive", "cam/img.jpg"),
  };
  EXPECT_EQ(paths.size(), 3u);
  for (const auto& p : paths)
  {
    EXPECT_EQ(p.rfind(dir + "/", 0), 0u) << p;
    // An object key is not a legal filename; the hash is what makes it one.
    EXPECT_EQ(p.find("cam/"), std::string::npos) << p;
  }
  // Stable across calls — a resumed/replayed attempt reuses (and so overwrites) the same
  // scratch file rather than leaking a new one each time.
  EXPECT_EQ(thumbnail_scratch_path(dir, "minio", "cam/img.jpg"), thumbnail_scratch_path(dir, "minio", "cam/img.jpg"));
  // A trailing separator on the state dir doesn't produce a doubled slash.
  const std::string key = "cam/img.jpg";
  EXPECT_EQ(thumbnail_scratch_path(dir + "/", "minio", key), thumbnail_scratch_path(dir, "minio", key));
}

TEST(Thumbnail, FfmpegGeneratorReportsFailureRatherThanThrowingWhenTheBinaryIsAbsent)
{
  // The contract the Uploader relies on: a generator never throws, and "no ffmpeg on
  // PATH" is an ordinary false — that is what keeps a missing decode dependency from
  // touching the primary upload at all.
  auto gen = ffmpeg_thumbnail_generator("dc-no-such-ffmpeg-binary");
  EXPECT_FALSE(gen("/nonexistent/source.jpg", "/tmp/dc-thumb-should-not-exist.jpg", 320));
}

TEST(Thumbnail, FfmpegGeneratorReportsFailureOnAnUndecodableFile)
{
  auto tmp = std::filesystem::temp_directory_path() / ("dc_thumb_" + std::to_string(::getpid()));
  std::filesystem::create_directories(tmp);
  const auto junk = tmp / "not-an-image.bin";
  std::ofstream(junk, std::ios::binary) << "definitely not pixels";

  auto gen = ffmpeg_thumbnail_generator("ffmpeg");
  EXPECT_FALSE(gen(junk.string(), (tmp / "out.jpg").string(), 320));
  std::filesystem::remove_all(tmp);
}

namespace
{

/// (width, height) from a JPEG's SOF marker, or (0, 0) if it isn't one. Enough of a
/// parser to assert what the scale filter actually did, without a decode dependency in
/// the test itself.
std::pair<int, int> jpeg_dimensions(const std::string& bytes)
{
  if (bytes.size() < 4 || static_cast<unsigned char>(bytes[0]) != 0xFF || static_cast<unsigned char>(bytes[1]) != 0xD8)
  {
    return { 0, 0 };
  }
  std::size_t i = 2;
  while (i + 3 < bytes.size())
  {
    if (static_cast<unsigned char>(bytes[i]) != 0xFF)
    {
      return { 0, 0 };
    }
    const unsigned char marker = static_cast<unsigned char>(bytes[i + 1]);
    const std::size_t len = (static_cast<unsigned char>(bytes[i + 2]) << 8) | static_cast<unsigned char>(bytes[i + 3]);
    // SOF0..SOF15, excluding the DHT/JPG/DAC markers interleaved in that range.
    const bool is_sof = marker >= 0xC0 && marker <= 0xCF && marker != 0xC4 && marker != 0xC8 && marker != 0xCC;
    if (is_sof && i + 9 < bytes.size())
    {
      const int h = (static_cast<unsigned char>(bytes[i + 5]) << 8) | static_cast<unsigned char>(bytes[i + 6]);
      const int w = (static_cast<unsigned char>(bytes[i + 7]) << 8) | static_cast<unsigned char>(bytes[i + 8]);
      return { w, h };
    }
    i += 2 + len;
  }
  return { 0, 0 };
}

std::string read_all(const std::filesystem::path& p)
{
  std::ifstream in(p, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

}  // namespace

// Real ffmpeg, when the machine has it — the ffmpeg argument vector is otherwise the one
// part of this feature no fake can check. Skipped rather than failed where ffmpeg is
// absent: dc_bridge builds and its other tests run with no media toolchain at all
// (package.xml declares ffmpeg as an exec_depend, not a build one), which is the same
// reason the Uploader treats a missing binary as "no preview" rather than an error.
TEST(Thumbnail, FfmpegGeneratorProducesABoundedJpegAndNeverUpscales)
{
  auto gen = ffmpeg_thumbnail_generator("ffmpeg");
  auto tmp = std::filesystem::temp_directory_path() / ("dc_thumb_real_" + std::to_string(::getpid()));
  std::filesystem::create_directories(tmp);

  // A 40x20 binary Netpbm greyscale — the same family as a DC nav map's PGM, and simple
  // enough to write by hand so the test needs no fixture file.
  const auto pgm = tmp / "src.pgm";
  {
    std::ofstream out(pgm, std::ios::binary);
    out << "P5\n40 20\n255\n";
    for (int i = 0; i < 40 * 20; ++i)
    {
      out.put(static_cast<char>(i % 256));
    }
  }

  const auto scaled = tmp / "scaled.jpg";
  if (!gen(pgm.string(), scaled.string(), 10))
  {
    std::filesystem::remove_all(tmp);
    GTEST_SKIP() << "ffmpeg not available";
  }

  const std::string bytes = read_all(scaled);
  // JPEG regardless of what went in — content_type::sniff would call this image/jpeg.
  ASSERT_EQ(bytes.substr(0, 3), std::string("\xff\xd8\xff", 3));
  // Bounded on the *longest* side, aspect ratio preserved: 40x20 into a 10-px bound.
  EXPECT_EQ(jpeg_dimensions(bytes), std::make_pair(10, 5));

  // The same source under a bound far above its own size comes back at its original
  // size, not blown up — min(source, bound) is what makes "thumbnail" mean "no larger".
  const auto unscaled = tmp / "unscaled.jpg";
  ASSERT_TRUE(gen(pgm.string(), unscaled.string(), 4096));
  EXPECT_EQ(jpeg_dimensions(read_all(unscaled)), std::make_pair(40, 20));

  std::filesystem::remove_all(tmp);
}
