// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Ports of the inline readiness / config (TopicConfig) / vector_binary tests, plus
// atomic_write (#444) and net_resolve (#445).
#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "dc_bridge/atomic_write.hpp"
#include "dc_bridge/net_resolve.hpp"
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

TEST(Readiness, ProbeAcceptsAHostnameNotOnlyALiteralIp)
{
  // #445: a container-network peer's readiness (e.g. a Compose service) is checked by
  // name, not only ever by a literal IP — this must resolve, not fail at parse time.
  int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  sa.sin_port = 0;
  ::bind(listen_fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa));
  ::listen(listen_fd, 1);
  socklen_t len = sizeof(sa);
  ::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&sa), &len);
  const std::uint16_t port = ntohs(sa.sin_port);

  EXPECT_TRUE(probe("localhost", port, std::chrono::milliseconds(500)));

  ::close(listen_fd);
}

TEST(NetResolve, ResolvesALiteralIPv4Address)
{
  sockaddr_in sa{};
  ASSERT_TRUE(resolve_ipv4("127.0.0.1", 4242, sa));
  EXPECT_EQ(sa.sin_addr.s_addr, htonl(INADDR_LOOPBACK));
  EXPECT_EQ(sa.sin_port, htons(4242));
}

TEST(NetResolve, ResolvesAHostname)
{
  sockaddr_in sa{};
  ASSERT_TRUE(resolve_ipv4("localhost", 4242, sa));
  EXPECT_EQ(sa.sin_addr.s_addr, htonl(INADDR_LOOPBACK));
  EXPECT_EQ(sa.sin_port, htons(4242));
}

TEST(NetResolve, FailsOnAHostnameThatDoesNotResolve)
{
  sockaddr_in sa{};
  EXPECT_FALSE(resolve_ipv4("this-host-does-not-exist.invalid", 4242, sa));
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

namespace
{
std::string read_file(const std::filesystem::path& path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
}  // namespace

TEST(AtomicWrite, WritesContentAndLeavesNoTmpFileBehind)
{
  auto path =
      std::filesystem::temp_directory_path() / ("dc_bridge_atomic_write_test_" + std::to_string(::getpid()) + ".toml");
  std::filesystem::remove(path);

  write_file_atomically(path.string(), "hello world");

  EXPECT_EQ(read_file(path), "hello world");
  EXPECT_FALSE(std::filesystem::exists(path.string() + ".tmp"));

  std::filesystem::remove(path);
}

TEST(AtomicWrite, OverwritesAnExistingFileCompletely)
{
  auto path = std::filesystem::temp_directory_path() /
              ("dc_bridge_atomic_write_test_" + std::to_string(::getpid()) + "_overwrite.toml");
  std::ofstream(path) << "a much longer previous config that should not leave any trailing bytes behind";

  write_file_atomically(path.string(), "short");

  EXPECT_EQ(read_file(path), "short");

  std::filesystem::remove(path);
}

TEST(AtomicWrite, ThrowsWhenTheDirectoryDoesNotExist)
{
  auto path = std::filesystem::temp_directory_path() / ("dc_bridge_atomic_write_test_" + std::to_string(::getpid())) /
              "nonexistent" / "config.toml";
  EXPECT_THROW(write_file_atomically(path.string(), "content"), std::runtime_error);
}
