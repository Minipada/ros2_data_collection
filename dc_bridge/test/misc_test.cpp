// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Ports of the inline readiness / config (TopicConfig) / vector_binary tests.
#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "dc_bridge/readiness.hpp"
#include "dc_bridge/topic_config.hpp"
#include "dc_bridge/vector_binary.hpp"

using namespace dc_bridge;

TEST(Readiness, StartsNotReadyAndReflectsSetReady)
{
  Readiness r;
  EXPECT_FALSE(r.is_ready());
  r.set_ready(true);
  EXPECT_TRUE(r.is_ready());
  r.set_ready(false);
  EXPECT_FALSE(r.is_ready());
}

TEST(Readiness, CopiesShareTheSameState)
{
  Readiness r;
  Readiness copy = r;
  r.set_ready(true);
  EXPECT_TRUE(copy.is_ready());
}

TEST(Readiness, ProbeFailsOnAClosedPort)
{
  // Nothing listening on this port → not ready.
  EXPECT_FALSE(probe("127.0.0.1", 1, std::chrono::milliseconds(100)));
}

TEST(TopicConfig, DerivesDottedTagFromTopicName)
{
  EXPECT_EQ(TopicConfig::make("/dc/measurement/uptime").tag, "dc.measurement.uptime");
}

TEST(TopicConfig, ExplicitTagOverridesDerived)
{
  EXPECT_EQ(TopicConfig::make("/dc/measurement/uptime", "custom.tag").tag, "custom.tag");
}

TEST(TopicConfig, RootTopicFallsBackToDefaultTag)
{
  EXPECT_EQ(TopicConfig::make("/").tag, "dc");
}

TEST(VectorBinary, FindsBinaryInFirstPrefixThatHasIt)
{
  auto base = std::filesystem::temp_directory_path() / ("dc_bridge_vb_test_" + std::to_string(::getpid()));
  auto empty_prefix = base / "empty";
  auto real_prefix = base / "real";
  auto vector_dir = real_prefix / "lib" / "vector_vendor";
  std::filesystem::create_directories(empty_prefix);
  std::filesystem::create_directories(vector_dir);
  std::ofstream(vector_dir / "vector") << "#!/bin/sh\n";

  std::string amentp = empty_prefix.string() + ":" + real_prefix.string();
  auto found = find_vector_binary(amentp);
  std::filesystem::remove_all(base);

  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(*found, (vector_dir / "vector").string());
}

TEST(VectorBinary, ReturnsNoneWhenNoPrefixHasBinary)
{
  EXPECT_FALSE(find_vector_binary("/nonexistent/prefix").has_value());
}
