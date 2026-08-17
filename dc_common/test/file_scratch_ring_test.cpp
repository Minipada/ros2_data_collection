// Unit tests for dc_common::FileScratchRing (#285, part of #282's pre-event circular-buffer
// capture). No node, no rclcpp::init(): exercised purely against a tmp-dir fixture (RAII
// cleanup per test), following the pattern used in dc_bridge/test/retention_test.cpp.

#include "dc_common/file_scratch_ring.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using dc_common::FileScratchRing;
using dc_common::ScratchedFile;

namespace
{

std::chrono::system_clock::time_point at(int seconds)
{
  return std::chrono::system_clock::time_point(std::chrono::seconds(seconds));
}

struct Fixture
{
  std::filesystem::path tmp;
  std::filesystem::path sources;
  std::filesystem::path scratch;

  Fixture()
  {
    tmp = std::filesystem::temp_directory_path() /
          ("dc_file_scratch_ring_test_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    sources = tmp / "sources";
    scratch = tmp / "scratch";
    std::filesystem::create_directories(sources);
  }
  ~Fixture()
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp, ec);
  }

  std::filesystem::path write_source(const std::string& name, const std::string& contents = "x")
  {
    auto p = sources / name;
    std::ofstream(p, std::ios::binary) << contents;
    return p;
  }

  static std::atomic<int> counter_;
};
std::atomic<int> Fixture::counter_{ 0 };

std::vector<std::filesystem::path> paths_of(const std::vector<ScratchedFile>& entries)
{
  std::vector<std::filesystem::path> out;
  out.reserve(entries.size());
  for (const auto& e : entries)
  {
    out.push_back(e.path);
  }
  return out;
}

}  // namespace

TEST(FileScratchRing, ConstructorCreatesTheScratchDirectory)
{
  Fixture fx;
  ASSERT_FALSE(std::filesystem::exists(fx.scratch));

  FileScratchRing ring(fx.scratch, std::chrono::seconds(60));

  EXPECT_TRUE(std::filesystem::exists(fx.scratch));
  EXPECT_TRUE(std::filesystem::is_directory(fx.scratch));
}

TEST(FileScratchRing, StageCopiesTheFileAndLeavesTheSourceInPlace)
{
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(60));
  auto source = fx.write_source("img.jpg", "hello");

  auto staged = ring.stage(source, at(0));

  EXPECT_TRUE(std::filesystem::exists(source));  // stage() copies, it doesn't move.
  ASSERT_TRUE(std::filesystem::exists(staged));
  EXPECT_NE(staged, source);
  std::ifstream in(staged, std::ios::binary);
  std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_EQ(contents, "hello");
}

TEST(FileScratchRing, StagingTheSameSourceTwiceProducesDistinctStagedPaths)
{
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(60));
  auto source = fx.write_source("img.jpg");

  auto first = ring.stage(source, at(0));
  auto second = ring.stage(source, at(1));

  EXPECT_NE(first, second);
  EXPECT_TRUE(std::filesystem::exists(first));
  EXPECT_TRUE(std::filesystem::exists(second));
  EXPECT_EQ(ring.size(), 2u);
}

TEST(FileScratchRing, WindowReturnsEntriesInTimeOrderRegardlessOfStageOrder)
{
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(60));
  auto middle = ring.stage(fx.write_source("middle.jpg"), at(5));
  auto first = ring.stage(fx.write_source("first.jpg"), at(1));
  auto last = ring.stage(fx.write_source("last.jpg"), at(9));

  EXPECT_EQ(paths_of(ring.window()), std::vector<std::filesystem::path>({ first, middle, last }));
}

TEST(FileScratchRing, EvictDeletesEntriesStrictlyOlderThanWindowFromDiskAndWindow)
{
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(10));
  auto old_path = ring.stage(fx.write_source("old.jpg"), at(0));
  auto new_path = ring.stage(fx.write_source("new.jpg"), at(5));

  ring.evict(at(11));  // "old" is 11s old (>10s window); "new" is 6s old (<=10s window).

  EXPECT_EQ(ring.size(), 1u);
  EXPECT_FALSE(std::filesystem::exists(old_path));
  EXPECT_TRUE(std::filesystem::exists(new_path));
  EXPECT_EQ(paths_of(ring.window()), std::vector<std::filesystem::path>({ new_path }));
}

TEST(FileScratchRing, EvictBoundaryAgeEqualToWindowIsKept)
{
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(10));
  auto staged = ring.stage(fx.write_source("boundary.jpg"), at(0));

  ring.evict(at(10));  // age == window exactly: inclusive boundary, must survive.

  EXPECT_EQ(ring.size(), 1u);
  EXPECT_TRUE(std::filesystem::exists(staged));
}

TEST(FileScratchRing, EvictBoundaryOneTickPastWindowIsDropped)
{
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(10));
  auto staged = ring.stage(fx.write_source("boundary.jpg"), at(0));

  ring.evict(at(11));  // age > window by the smallest margin used here: must be dropped.

  EXPECT_TRUE(ring.empty());
  EXPECT_FALSE(std::filesystem::exists(staged));
}

TEST(FileScratchRing, EvictOnEmptyRingIsANoop)
{
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(10));
  ring.evict(at(1000));
  EXPECT_TRUE(ring.empty());
}

TEST(FileScratchRing, EvictToleratesAStagedFileAlreadyMissingFromDisk)
{
  // Simulates something else on the system removing a staged copy out of band: the ring must
  // still converge cleanly on the next evict rather than throwing or getting stuck.
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(10));
  auto staged = ring.stage(fx.write_source("gone.jpg"), at(0));
  std::filesystem::remove(staged);

  ring.evict(at(11));

  EXPECT_TRUE(ring.empty());
}

TEST(FileScratchRing, ReleaseHandsTheWindowOnWithoutDeletingAnything)
{
  // #290: the released Files belong to the Bridge's Files pipeline from here on, so release()
  // must forget them without touching disk -- and a later evict() must not delete them either.
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(10));
  auto first = ring.stage(fx.write_source("first.jpg"), at(0));
  auto second = ring.stage(fx.write_source("second.jpg"), at(1));

  auto released = ring.release();

  EXPECT_EQ(paths_of(released), std::vector<std::filesystem::path>({ first, second }));
  EXPECT_TRUE(ring.empty());
  EXPECT_TRUE(std::filesystem::exists(first));
  EXPECT_TRUE(std::filesystem::exists(second));

  ring.evict(at(10000));  // long past the window: the ring no longer knows about them.
  EXPECT_TRUE(std::filesystem::exists(first));
  EXPECT_TRUE(std::filesystem::exists(second));
}

TEST(FileScratchRing, ReleaseOnEmptyRingReturnsNothing)
{
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(10));
  EXPECT_TRUE(ring.release().empty());
  EXPECT_TRUE(ring.empty());
}

TEST(FileScratchRing, PurgeDeletesEveryStagedFileWhateverItsAge)
{
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(10));
  auto old_path = ring.stage(fx.write_source("old.jpg"), at(0));
  auto new_path = ring.stage(fx.write_source("new.jpg"), at(9));

  ring.purge();  // no "now": nothing staged survives, however fresh.

  EXPECT_TRUE(ring.empty());
  EXPECT_FALSE(std::filesystem::exists(old_path));
  EXPECT_FALSE(std::filesystem::exists(new_path));
}

TEST(FileScratchRing, PurgeToleratesAStagedFileAlreadyMissingFromDisk)
{
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(10));
  auto staged = ring.stage(fx.write_source("gone.jpg"), at(0));
  std::filesystem::remove(staged);

  ring.purge();

  EXPECT_TRUE(ring.empty());
}

TEST(FileScratchRing, StageThrowsWhenSourceFileDoesNotExist)
{
  Fixture fx;
  FileScratchRing ring(fx.scratch, std::chrono::seconds(10));

  EXPECT_THROW(ring.stage(fx.sources / "does_not_exist.jpg", at(0)), std::filesystem::filesystem_error);
  EXPECT_TRUE(ring.empty());
}
