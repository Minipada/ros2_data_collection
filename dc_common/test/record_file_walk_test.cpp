// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Unit tests for dc_common::record_file_walk (#479): the shared owner of "where Files live in
// a Record". The flattened forms below are exactly what nlohmann's json::flatten() produces
// (leading-slash JSON-pointer keys), which is what Measurement::flattenSample() runs on every
// Record -- these tests pin both written forms so the staging side (dc_measurements) and the
// parse side (dc_bridge) can never drift apart again.

#include "dc_common/record_file_walk.hpp"

#include <gtest/gtest.h>

#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using dc_common::record_file_walk::for_each_file_site;
using dc_common::record_file_walk::rewrite_local_paths;
using nlohmann::json;

namespace
{

struct Site
{
  std::string key;
  std::string local_path;
  std::map<std::string, std::string> remote_paths;
};

std::vector<Site> sites(const json& record)
{
  std::vector<Site> out;
  for_each_file_site(record, [&](const dc_common::record_file_walk::FileSite& s) {
    out.push_back({ s.key, s.local_path, s.remote_paths });
  });
  return out;
}

}  // namespace

TEST(RecordFileWalk, NestedTopLevelLocalPaths)
{
  const json record = json::parse(R"({
    "local_paths": {"raw": "/tmp/raw.jpg"},
    "remote_paths": {"s3": {"raw": "s3://bucket/raw.jpg"}}
  })");
  const auto found = sites(record);
  ASSERT_EQ(found.size(), 1u);
  EXPECT_EQ(found[0].key, "raw");
  EXPECT_EQ(found[0].local_path, "/tmp/raw.jpg");
  EXPECT_EQ(found[0].remote_paths, (std::map<std::string, std::string>{ { "s3", "s3://bucket/raw.jpg" } }));
}

TEST(RecordFileWalk, NestedUnderMeasurementName)
{
  // nested: true wraps the payload under the measurement name before flatten is even consulted.
  const json record = json::parse(R"({
    "camera": {
      "local_paths": {"raw": "/tmp/raw.jpg", "rotated": "/tmp/rot.jpg"},
      "remote_paths": {"minio": {"raw": "minio://raw.jpg"}}
    },
    "nested": true,
    "flattened": false
  })");
  const auto found = sites(record);
  ASSERT_EQ(found.size(), 2u);
  EXPECT_EQ(found[0].key, "raw");
  EXPECT_EQ(found[0].remote_paths, (std::map<std::string, std::string>{ { "minio", "minio://raw.jpg" } }));
  EXPECT_EQ(found[1].key, "rotated");
  EXPECT_TRUE(found[1].remote_paths.empty());
}

TEST(RecordFileWalk, FlattenedTopLevel)
{
  // Exactly json::flatten() output for the nested-top-level record above, plus the marker
  // keys flattenSample() adds afterwards. Pre-fix (#479) the Bridge's walk missed these.
  const json record = json::parse(R"({
    "/local_paths/raw": "/tmp/raw.jpg",
    "/remote_paths/s3/raw": "s3://bucket/raw.jpg",
    "/local_paths/rotated": "/tmp/rot.jpg",
    "flattened": true,
    "nested": false
  })");
  const auto found = sites(record);
  ASSERT_EQ(found.size(), 2u);
  EXPECT_EQ(found[0].key, "raw");
  EXPECT_EQ(found[0].local_path, "/tmp/raw.jpg");
  EXPECT_EQ(found[0].remote_paths, (std::map<std::string, std::string>{ { "s3", "s3://bucket/raw.jpg" } }));
  EXPECT_EQ(found[1].key, "rotated");
  EXPECT_EQ(found[1].local_path, "/tmp/rot.jpg");
}

TEST(RecordFileWalk, FlattenedUnderMeasurementName)
{
  // nested + flatten: pointer keys carry the measurement name as their first segment.
  const json record = json::parse(R"({
    "/camera/local_paths/raw": "/tmp/raw.jpg",
    "/camera/remote_paths/s3/raw": "s3://bucket/raw.jpg",
    "/other/local_paths/aux": "/tmp/aux.dat",
    "flattened": true,
    "nested": true
  })");
  const auto found = sites(record);
  ASSERT_EQ(found.size(), 2u);
  EXPECT_EQ(found[0].key, "raw");
  EXPECT_EQ(found[0].local_path, "/tmp/raw.jpg");
  EXPECT_EQ(found[0].remote_paths, (std::map<std::string, std::string>{ { "s3", "s3://bucket/raw.jpg" } }));
  // A different measurement's site in the same flat object must not steal camera's remotes.
  EXPECT_EQ(found[1].key, "aux");
  EXPECT_TRUE(found[1].remote_paths.empty());
}

TEST(RecordFileWalk, SkipsBase64EmptyAndNonStringEntries)
{
  const json record = json::parse(R"({
    "local_paths": {"raw": "/tmp/raw.jpg", "empty": "", "inline": 42},
    "base64": {"small": "aGVsbG8="},
    "n": 1
  })");
  const auto found = sites(record);
  ASSERT_EQ(found.size(), 1u);
  EXPECT_EQ(found[0].key, "raw");
}

TEST(RecordFileWalk, DropsEmptyRemotesAndKeepsSiteWithoutRemotePaths)
{
  const json record = json::parse(R"({
    "local_paths": {"raw": "/tmp/raw.jpg"},
    "remote_paths": {"s3": {"raw": ""}, "other": {"raw": "s3://ok"}}
  })");
  const auto found = sites(record);
  ASSERT_EQ(found.size(), 1u);
  EXPECT_EQ(found[0].remote_paths, (std::map<std::string, std::string>{ { "other", "s3://ok" } }));
}

TEST(RecordFileWalk, NoFilesInPlainRecord)
{
  EXPECT_TRUE(sites(json::parse(R"({"cpu": 0.4, "tags": ["a"]})")).empty());
}

TEST(RecordFileWalk, RewritesNestedLocalPathsInPlace)
{
  json record = json::parse(R"({
    "camera": {"local_paths": {"raw": "/tmp/raw.jpg"}, "remote_paths": {"s3": {"raw": "s3://r"}}},
    "local_paths": {"map": "/tmp/map.pgm"}
  })");
  const bool any = rewrite_local_paths(record, [&](std::string& path) {
    path = "/staged" + path;
    return true;
  });
  EXPECT_TRUE(any);
  EXPECT_EQ(record["camera"]["local_paths"]["raw"], "/staged/tmp/raw.jpg");
  EXPECT_EQ(record["local_paths"]["map"], "/staged/tmp/map.pgm");
  // remotes untouched
  EXPECT_EQ(record["camera"]["remote_paths"]["s3"]["raw"], "s3://r");
}

TEST(RecordFileWalk, RewritesFlattenedLocalPathsInPlace)
{
  json record = json::parse(R"({
    "/camera/local_paths/raw": "/tmp/raw.jpg",
    "/camera/remote_paths/s3/raw": "s3://r",
    "/local_paths/map": "/tmp/map.pgm"
  })");
  const bool any = rewrite_local_paths(record, [&](std::string& path) {
    path = "/staged" + path;
    return true;
  });
  EXPECT_TRUE(any);
  EXPECT_EQ(record["/camera/local_paths/raw"], "/staged/tmp/raw.jpg");
  EXPECT_EQ(record["/local_paths/map"], "/staged/tmp/map.pgm");
  EXPECT_EQ(record["/camera/remote_paths/s3/raw"], "s3://r");
}

TEST(RecordFileWalk, RewriteReturnsFalseWhenNothingRewritten)
{
  json record = json::parse(R"({"cpu": 0.4})");
  EXPECT_FALSE(rewrite_local_paths(record, [&](std::string&) { return true; }));

  json no_op = json::parse(R"({"local_paths": {"raw": "/tmp/raw.jpg"}})");
  EXPECT_FALSE(rewrite_local_paths(no_op, [&](std::string&) { return false; }));
  EXPECT_EQ(no_op["local_paths"]["raw"], "/tmp/raw.jpg");
}
