// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Unit tests for dc_measurements::MissionNav2ThroughPosesCore (#388). No node, no rclcpp::init():
// goal_id/status/result all arrive as arguments, mirroring dc_common::BatteryCycleAccumulator's
// standalone coverage.

#include <gtest/gtest.h>

#include <chrono>
#include <optional>
#include <string>

#include "dc_measurements/plugins/measurements/mission_nav2_through_poses_core.hpp"

using dc_measurements::GoalPhase;
using dc_measurements::MissionNav2ThroughPosesCore;
using dc_measurements::MissionOutcome;
using dc_measurements::WaypointStatusCounts;

namespace
{

std::chrono::system_clock::time_point at(int seconds)
{
  return std::chrono::system_clock::time_point(std::chrono::seconds(seconds));
}

}  // namespace

TEST(MissionNav2ThroughPosesCore, FirstSampleOfAGoalIdIsAMissionStart)
{
  MissionNav2ThroughPosesCore core;
  const auto start = core.observe("goal-1", at(0));
  ASSERT_TRUE(start.has_value());
  EXPECT_EQ(start->mission_id, "goal-1");
  EXPECT_EQ(start->sequence, 1u);
}

TEST(MissionNav2ThroughPosesCore, RepeatedSamplesOfTheSameGoalIdProduceNoFurtherStart)
{
  MissionNav2ThroughPosesCore core;
  ASSERT_TRUE(core.observe("goal-1", at(0)).has_value());
  EXPECT_FALSE(core.observe("goal-1", at(1)).has_value());
  EXPECT_FALSE(core.observe("goal-1", at(2)).has_value());
}

TEST(MissionNav2ThroughPosesCore, TwoDifferentGoalIdsEachGetTheirOwnStart)
{
  MissionNav2ThroughPosesCore core;
  const auto first = core.observe("goal-1", at(0));
  const auto second = core.observe("goal-2", at(1));
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->sequence, 1u);
  EXPECT_EQ(second->sequence, 2u);
}

TEST(MissionNav2ThroughPosesCore, ShouldRequestResultIsTrueOnlyOncePerGoalId)
{
  MissionNav2ThroughPosesCore core;
  core.observe("goal-1", at(0));
  EXPECT_TRUE(core.shouldRequestResult("goal-1"));
  EXPECT_FALSE(core.shouldRequestResult("goal-1"));
}

TEST(MissionNav2ThroughPosesCore, ShouldRequestResultIsFalseForAnUntrackedGoalId)
{
  MissionNav2ThroughPosesCore core;
  EXPECT_FALSE(core.shouldRequestResult("never-seen"));
}

TEST(MissionNav2ThroughPosesCore, SucceededWithNoErrorCodeIsOutcomeSucceeded)
{
  MissionNav2ThroughPosesCore core;
  core.observe("goal-1", at(0));
  const auto fact = core.end("goal-1", GoalPhase::Succeeded, 0, "", std::nullopt, std::nullopt, at(10));

  EXPECT_EQ(fact.mission_id, "goal-1");
  EXPECT_EQ(fact.sequence, 2u);
  EXPECT_EQ(fact.outcome, MissionOutcome::Succeeded);
  EXPECT_DOUBLE_EQ(fact.duration_sec, 10.0);
  EXPECT_FALSE(fact.reason.has_value());
  EXPECT_FALSE(fact.error_code.has_value());
}

TEST(MissionNav2ThroughPosesCore, SucceededWithNonZeroErrorCodeIsOutcomeFailed)
{
  MissionNav2ThroughPosesCore core;
  core.observe("goal-1", at(0));
  const auto fact =
      core.end("goal-1", GoalPhase::Succeeded, 9100, "unknown failure", std::nullopt, std::nullopt, at(5));

  EXPECT_EQ(fact.outcome, MissionOutcome::Failed);
  ASSERT_TRUE(fact.reason.has_value());
  EXPECT_EQ(*fact.reason, "unknown failure");
  ASSERT_TRUE(fact.error_code.has_value());
  EXPECT_EQ(*fact.error_code, 9100);
}

TEST(MissionNav2ThroughPosesCore, AbortedCarriesReasonAndErrorCode)
{
  MissionNav2ThroughPosesCore core;
  core.observe("goal-1", at(0));
  const auto fact = core.end("goal-1", GoalPhase::Aborted, 9102, "tf timeout", std::nullopt, std::nullopt, at(3));

  EXPECT_EQ(fact.outcome, MissionOutcome::Aborted);
  ASSERT_TRUE(fact.reason.has_value());
  EXPECT_EQ(*fact.reason, "tf timeout");
  ASSERT_TRUE(fact.error_code.has_value());
  EXPECT_EQ(*fact.error_code, 9102);
}

TEST(MissionNav2ThroughPosesCore, CanceledCarriesNoReasonOrErrorCode)
{
  MissionNav2ThroughPosesCore core;
  core.observe("goal-1", at(0));
  const auto fact = core.end("goal-1", GoalPhase::Canceled, 0, "", std::nullopt, std::nullopt, at(2));

  EXPECT_EQ(fact.outcome, MissionOutcome::Cancelled);
  EXPECT_FALSE(fact.reason.has_value());
  EXPECT_FALSE(fact.error_code.has_value());
}

TEST(MissionNav2ThroughPosesCore, RecoveriesAndWaypointStatusesAreCarriedThrough)
{
  MissionNav2ThroughPosesCore core;
  core.observe("goal-1", at(0));

  WaypointStatusCounts counts;
  counts.total = 3;
  counts.completed = 2;
  counts.skipped = 0;
  counts.failed = 1;

  const auto fact = core.end("goal-1", GoalPhase::Succeeded, 0, "", 2, counts, at(1));

  ASSERT_TRUE(fact.recoveries.has_value());
  EXPECT_EQ(*fact.recoveries, 2);
  ASSERT_TRUE(fact.waypoint_statuses.has_value());
  EXPECT_EQ(fact.waypoint_statuses->total, 3u);
  EXPECT_EQ(fact.waypoint_statuses->completed, 2u);
  EXPECT_EQ(fact.waypoint_statuses->failed, 1u);
}

TEST(MissionNav2ThroughPosesCore, OpenMissionIdsReportsOnlyUnfinishedMissions)
{
  MissionNav2ThroughPosesCore core;
  core.observe("goal-1", at(0));
  core.observe("goal-2", at(1));
  core.end("goal-1", GoalPhase::Succeeded, 0, "", std::nullopt, std::nullopt, at(2));

  const auto open = core.openMissionIds();
  ASSERT_EQ(open.size(), 1u);
  EXPECT_EQ(open.front(), "goal-2");
}

TEST(MissionNav2ThroughPosesCore, SequenceIsMonotonicAcrossStartsAndEndsOfSeveralMissions)
{
  MissionNav2ThroughPosesCore core;
  const auto start1 = core.observe("goal-1", at(0));
  const auto end1 = core.end("goal-1", GoalPhase::Succeeded, 0, "", std::nullopt, std::nullopt, at(1));
  const auto start2 = core.observe("goal-2", at(2));
  const auto end2 = core.end("goal-2", GoalPhase::Aborted, 9102, "tf timeout", std::nullopt, std::nullopt, at(3));

  EXPECT_EQ(start1->sequence, 1u);
  EXPECT_EQ(end1.sequence, 2u);
  EXPECT_EQ(start2->sequence, 3u);
  EXPECT_EQ(end2.sequence, 4u);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
