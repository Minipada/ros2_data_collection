// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Unit tests for dc_measurements::MissionNav2Tracker (#387). No action server, no node: goal_id/
// status/result all arrive as arguments, mirroring
// dc_common/test/battery_cycle_accumulator_test.cpp's standalone coverage of
// BatteryCycleAccumulator. rclcpp::init()/shutdown() in main() below is only for link
// compatibility with this package's CMakeLists.txt test registration, which does not pass
// SKIP_LINKING_MAIN_LIBRARIES to ament_add_gtest.

#include "dc_measurements/plugins/measurements/mission_nav2_tracker.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

using dc_measurements::MissionNav2Tracker;
using dc_measurements::MissionOutcome;
using dc_measurements::MissionTerminalStatus;

namespace
{

std::chrono::system_clock::time_point at(int seconds)
{
  return std::chrono::system_clock::time_point(std::chrono::seconds(seconds));
}

}  // namespace

TEST(MissionNav2Tracker, StartsIdle)
{
  MissionNav2Tracker tracker(at(0));
  EXPECT_FALSE(tracker.activeMissionId().has_value());
}

TEST(MissionNav2Tracker, AcceptedGoalProducesAMissionStart)
{
  MissionNav2Tracker tracker(at(0));

  const auto start = tracker.startMission("goal-1", at(5));
  ASSERT_TRUE(start.has_value());
  EXPECT_EQ(start->mission_id, "goal-1");
  EXPECT_EQ(start->sequence, 1u);
  ASSERT_TRUE(tracker.activeMissionId().has_value());
  EXPECT_EQ(*tracker.activeMissionId(), "goal-1");
}

TEST(MissionNav2Tracker, SecondGoalWhileOneIsActiveIsIgnored)
{
  MissionNav2Tracker tracker(at(0));
  ASSERT_TRUE(tracker.startMission("goal-1", at(5)).has_value());

  EXPECT_FALSE(tracker.startMission("goal-2", at(6)).has_value());
  EXPECT_EQ(*tracker.activeMissionId(), "goal-1");
}

TEST(MissionNav2Tracker, SucceededResultProducesAMissionEndWithNoReason)
{
  MissionNav2Tracker tracker(at(0));
  tracker.startMission("goal-1", at(5));

  const auto end = tracker.endMission("goal-1", MissionTerminalStatus::Succeeded, 0, "", std::nullopt, at(35));
  ASSERT_TRUE(end.has_value());
  EXPECT_EQ(end->mission_id, "goal-1");
  EXPECT_EQ(end->sequence, 2u);
  EXPECT_EQ(end->outcome, MissionOutcome::Succeeded);
  EXPECT_DOUBLE_EQ(end->duration_sec, 30.0);
  EXPECT_FALSE(end->reason.has_value());
  EXPECT_FALSE(end->error_code.has_value());
  EXPECT_FALSE(end->recoveries.has_value());
  EXPECT_FALSE(tracker.activeMissionId().has_value());
}

TEST(MissionNav2Tracker, SucceededStatusWithNonZeroErrorCodeIsReportedAsFailed)
{
  MissionNav2Tracker tracker(at(0));
  tracker.startMission("goal-1", at(5));

  const auto end = tracker.endMission("goal-1", MissionTerminalStatus::Succeeded, 9101, "failed to load behavior tree",
                                      std::nullopt, at(20));
  ASSERT_TRUE(end.has_value());
  EXPECT_EQ(end->outcome, MissionOutcome::Failed);
  ASSERT_TRUE(end->reason.has_value());
  EXPECT_EQ(*end->reason, "failed to load behavior tree");
  ASSERT_TRUE(end->error_code.has_value());
  EXPECT_EQ(*end->error_code, 9101u);
}

TEST(MissionNav2Tracker, CanceledStatusIsReportedAsCancelled)
{
  MissionNav2Tracker tracker(at(0));
  tracker.startMission("goal-1", at(0));

  const auto end = tracker.endMission("goal-1", MissionTerminalStatus::Canceled, 0, "", std::nullopt, at(10));
  ASSERT_TRUE(end.has_value());
  EXPECT_EQ(end->outcome, MissionOutcome::Cancelled);
  EXPECT_FALSE(end->reason.has_value());
  EXPECT_FALSE(end->error_code.has_value());
}

TEST(MissionNav2Tracker, AbortedStatusCarriesReasonAndErrorCode)
{
  MissionNav2Tracker tracker(at(0));
  tracker.startMission("goal-1", at(0));

  const auto end =
      tracker.endMission("goal-1", MissionTerminalStatus::Aborted, 9102, "tf timeout", std::nullopt, at(2));
  ASSERT_TRUE(end.has_value());
  EXPECT_EQ(end->outcome, MissionOutcome::Aborted);
  ASSERT_TRUE(end->reason.has_value());
  EXPECT_EQ(*end->reason, "tf timeout");
  ASSERT_TRUE(end->error_code.has_value());
  EXPECT_EQ(*end->error_code, 9102u);
}

TEST(MissionNav2Tracker, ResultForAForeignGoalIdIsIgnored)
{
  MissionNav2Tracker tracker(at(0));
  tracker.startMission("goal-1", at(0));

  EXPECT_FALSE(
      tracker.endMission("goal-stale", MissionTerminalStatus::Succeeded, 0, "", std::nullopt, at(10)).has_value());
  EXPECT_TRUE(tracker.activeMissionId().has_value());
}

TEST(MissionNav2Tracker, RecoveriesAreCarriedThrough)
{
  MissionNav2Tracker tracker(at(0));
  tracker.startMission("goal-1", at(0));

  const auto end = tracker.endMission("goal-1", MissionTerminalStatus::Succeeded, 0, "", 2, at(1));
  ASSERT_TRUE(end.has_value());
  ASSERT_TRUE(end->recoveries.has_value());
  EXPECT_EQ(*end->recoveries, 2);
}

TEST(MissionNav2Tracker, SequenceIncrementsAcrossMultipleMissions)
{
  MissionNav2Tracker tracker(at(0));

  const auto first_start = tracker.startMission("goal-1", at(0));
  const auto first_end = tracker.endMission("goal-1", MissionTerminalStatus::Succeeded, 0, "", std::nullopt, at(10));
  const auto second_start = tracker.startMission("goal-2", at(20));
  const auto second_end =
      tracker.endMission("goal-2", MissionTerminalStatus::Aborted, 9102, "tf timeout", std::nullopt, at(30));

  ASSERT_TRUE(first_start.has_value());
  ASSERT_TRUE(first_end.has_value());
  ASSERT_TRUE(second_start.has_value());
  ASSERT_TRUE(second_end.has_value());
  EXPECT_EQ(first_start->sequence, 1u);
  EXPECT_EQ(first_end->sequence, 2u);
  EXPECT_EQ(second_start->sequence, 3u);
  EXPECT_EQ(second_end->sequence, 4u);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
