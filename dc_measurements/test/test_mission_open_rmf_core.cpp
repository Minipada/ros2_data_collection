// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Unit tests for dc_measurements::MissionOpenRmfCore (#391). No node, no rclcpp::init(), no
// nlohmann::json: TaskStateSample fields arrive as arguments, mirroring #388's
// MissionNav2ThroughPosesCore standalone coverage.

#include <gtest/gtest.h>

#include <chrono>
#include <optional>
#include <string>

#include "dc_measurements/plugins/measurements/mission_open_rmf_core.hpp"

using dc_measurements::MissionOpenRmfCore;
using dc_measurements::MissionOutcome;
using dc_measurements::RmfTaskStatus;
using dc_measurements::TaskStateSample;

namespace
{

std::chrono::system_clock::time_point at(int seconds)
{
  return std::chrono::system_clock::time_point(std::chrono::seconds(seconds));
}

TaskStateSample sample(const std::string& id, RmfTaskStatus status, const std::string& type = "delivery")
{
  TaskStateSample s;
  s.mission_id = id;
  s.mission_type = type;
  s.status = status;
  return s;
}

}  // namespace

TEST(MissionOpenRmfCore, ParsesEveryRmfApiMsgsStatusString)
{
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("uninitialized"), RmfTaskStatus::Uninitialized);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("blocked"), RmfTaskStatus::Blocked);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("error"), RmfTaskStatus::Error);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("failed"), RmfTaskStatus::Failed);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("queued"), RmfTaskStatus::Queued);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("standby"), RmfTaskStatus::Standby);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("underway"), RmfTaskStatus::Underway);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("delayed"), RmfTaskStatus::Delayed);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("skipped"), RmfTaskStatus::Skipped);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("canceled"), RmfTaskStatus::Canceled);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("killed"), RmfTaskStatus::Killed);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("completed"), RmfTaskStatus::Completed);
  EXPECT_EQ(dc_measurements::parseRmfTaskStatus("some-future-value"), RmfTaskStatus::UnknownStatus);
}

TEST(MissionOpenRmfCore, UnderwayIsAMissionStart)
{
  MissionOpenRmfCore core;
  const auto result = core.observe(sample("task-1", RmfTaskStatus::Underway), at(0));
  ASSERT_TRUE(result.start.has_value());
  EXPECT_EQ(result.start->mission_id, "task-1");
  EXPECT_EQ(result.start->sequence, 1u);
  EXPECT_FALSE(result.end.has_value());
}

TEST(MissionOpenRmfCore, QueuedAndStandbyProduceNoStart)
{
  MissionOpenRmfCore core;
  EXPECT_FALSE(core.observe(sample("task-1", RmfTaskStatus::Queued), at(0)).start.has_value());
  EXPECT_FALSE(core.observe(sample("task-1", RmfTaskStatus::Standby), at(1)).start.has_value());
  EXPECT_TRUE(core.openMissionIds().empty());
}

TEST(MissionOpenRmfCore, RepeatedActiveSamplesProduceNoFurtherStart)
{
  MissionOpenRmfCore core;
  ASSERT_TRUE(core.observe(sample("task-1", RmfTaskStatus::Underway), at(0)).start.has_value());
  EXPECT_FALSE(core.observe(sample("task-1", RmfTaskStatus::Underway), at(1)).start.has_value());
  EXPECT_FALSE(core.observe(sample("task-1", RmfTaskStatus::Delayed), at(2)).start.has_value());
}

TEST(MissionOpenRmfCore, TwoDifferentMissionIdsEachGetTheirOwnStart)
{
  MissionOpenRmfCore core;
  const auto first = core.observe(sample("task-1", RmfTaskStatus::Underway), at(0));
  const auto second = core.observe(sample("task-2", RmfTaskStatus::Underway), at(1));
  ASSERT_TRUE(first.start.has_value());
  ASSERT_TRUE(second.start.has_value());
  EXPECT_EQ(first.start->sequence, 1u);
  EXPECT_EQ(second.start->sequence, 2u);
}

TEST(MissionOpenRmfCore, TerminalWithoutPriorStartProducesNoRecordsAtAll)
{
  MissionOpenRmfCore core;
  const auto result = core.observe(sample("task-1", RmfTaskStatus::Completed), at(0));
  EXPECT_FALSE(result.start.has_value());
  EXPECT_FALSE(result.end.has_value());
  EXPECT_TRUE(core.openMissionIds().empty());
}

TEST(MissionOpenRmfCore, BlockedAndErrorAreTransientNotTerminal)
{
  MissionOpenRmfCore core;
  ASSERT_TRUE(core.observe(sample("task-1", RmfTaskStatus::Underway), at(0)).start.has_value());
  EXPECT_FALSE(core.observe(sample("task-1", RmfTaskStatus::Blocked), at(1)).end.has_value());
  EXPECT_FALSE(core.observe(sample("task-1", RmfTaskStatus::Error), at(2)).end.has_value());
  EXPECT_EQ(core.openMissionIds().size(), 1u);

  const auto ended = core.observe(sample("task-1", RmfTaskStatus::Completed), at(3));
  ASSERT_TRUE(ended.end.has_value());
  EXPECT_EQ(ended.end->outcome, MissionOutcome::Succeeded);
  EXPECT_TRUE(core.openMissionIds().empty());
}

TEST(MissionOpenRmfCore, CompletedMapsToSucceeded)
{
  MissionOpenRmfCore core;
  core.observe(sample("task-1", RmfTaskStatus::Underway), at(0));
  const auto result = core.observe(sample("task-1", RmfTaskStatus::Completed), at(10));
  ASSERT_TRUE(result.end.has_value());
  EXPECT_EQ(result.end->mission_id, "task-1");
  EXPECT_EQ(result.end->sequence, 2u);
  EXPECT_EQ(result.end->outcome, MissionOutcome::Succeeded);
  EXPECT_DOUBLE_EQ(result.end->duration_sec, 10.0);
  EXPECT_FALSE(result.end->reason.has_value());
  EXPECT_FALSE(result.end->error_code.has_value());
}

TEST(MissionOpenRmfCore, FailedMapsToFailedWithReasonAndErrorCode)
{
  MissionOpenRmfCore core;
  core.observe(sample("task-1", RmfTaskStatus::Underway), at(0));
  auto failed = sample("task-1", RmfTaskStatus::Failed);
  failed.reason = "dispenser_unavailable: dispenser_1 did not respond";
  failed.error_code = 12;
  const auto result = core.observe(failed, at(5));
  ASSERT_TRUE(result.end.has_value());
  EXPECT_EQ(result.end->outcome, MissionOutcome::Failed);
  ASSERT_TRUE(result.end->reason.has_value());
  EXPECT_EQ(*result.end->reason, "dispenser_unavailable: dispenser_1 did not respond");
  ASSERT_TRUE(result.end->error_code.has_value());
  EXPECT_EQ(*result.end->error_code, 12u);
}

TEST(MissionOpenRmfCore, CanceledMapsToCancelledWithReason)
{
  MissionOpenRmfCore core;
  core.observe(sample("task-1", RmfTaskStatus::Underway), at(0));
  auto canceled = sample("task-1", RmfTaskStatus::Canceled);
  canceled.reason = "operator; dashboard";
  const auto result = core.observe(canceled, at(3));
  ASSERT_TRUE(result.end.has_value());
  EXPECT_EQ(result.end->outcome, MissionOutcome::Cancelled);
  ASSERT_TRUE(result.end->reason.has_value());
  EXPECT_EQ(*result.end->reason, "operator; dashboard");
}

TEST(MissionOpenRmfCore, KilledMapsToAbortedWithReason)
{
  MissionOpenRmfCore core;
  core.observe(sample("task-1", RmfTaskStatus::Underway), at(0));
  auto killed = sample("task-1", RmfTaskStatus::Killed);
  killed.reason = "safety_stop";
  const auto result = core.observe(killed, at(7));
  ASSERT_TRUE(result.end.has_value());
  EXPECT_EQ(result.end->outcome, MissionOutcome::Aborted);
  ASSERT_TRUE(result.end->reason.has_value());
  EXPECT_EQ(*result.end->reason, "safety_stop");
}

TEST(MissionOpenRmfCore, SkippedIsTerminalAndMapsToCancelled)
{
  MissionOpenRmfCore core;
  core.observe(sample("task-1", RmfTaskStatus::Underway), at(0));
  const auto result = core.observe(sample("task-1", RmfTaskStatus::Skipped), at(2));
  ASSERT_TRUE(result.end.has_value());
  EXPECT_EQ(result.end->outcome, MissionOutcome::Cancelled);
  EXPECT_TRUE(core.openMissionIds().empty());
}

TEST(MissionOpenRmfCore, RepeatedTerminalSamplesProduceNoDuplicateEnd)
{
  MissionOpenRmfCore core;
  core.observe(sample("task-1", RmfTaskStatus::Underway), at(0));
  ASSERT_TRUE(core.observe(sample("task-1", RmfTaskStatus::Completed), at(1)).end.has_value());
  EXPECT_FALSE(core.observe(sample("task-1", RmfTaskStatus::Completed), at(2)).end.has_value());
  EXPECT_FALSE(core.observe(sample("task-1", RmfTaskStatus::Completed), at(3)).start.has_value());
}

TEST(MissionOpenRmfCore, DurationPrefersUpstreamTimestampsOverLocallyObserved)
{
  MissionOpenRmfCore core;
  core.observe(sample("task-1", RmfTaskStatus::Underway), at(0));
  auto completed = sample("task-1", RmfTaskStatus::Completed);
  completed.unix_millis_start_time = 1000;
  completed.unix_millis_finish_time = 4500;
  // Observed 100s later locally, but the upstream timestamps (3.5s apart) should win.
  const auto result = core.observe(completed, at(100));
  ASSERT_TRUE(result.end.has_value());
  EXPECT_DOUBLE_EQ(result.end->duration_sec, 3.5);
}

TEST(MissionOpenRmfCore, DurationFallsBackToLocallyObservedTimestampsWhenUpstreamMissing)
{
  MissionOpenRmfCore core;
  core.observe(sample("task-1", RmfTaskStatus::Underway), at(0));
  const auto result = core.observe(sample("task-1", RmfTaskStatus::Completed), at(6));
  ASSERT_TRUE(result.end.has_value());
  EXPECT_DOUBLE_EQ(result.end->duration_sec, 6.0);
}

TEST(MissionOpenRmfCore, OpenMissionIdsReflectsStillActiveMissionsOnly)
{
  MissionOpenRmfCore core;
  core.observe(sample("task-1", RmfTaskStatus::Underway), at(0));
  core.observe(sample("task-2", RmfTaskStatus::Underway), at(1));
  core.observe(sample("task-1", RmfTaskStatus::Completed), at(2));

  const auto open = core.openMissionIds();
  ASSERT_EQ(open.size(), 1u);
  EXPECT_EQ(open.front(), "task-2");
}
