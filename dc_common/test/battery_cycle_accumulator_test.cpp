// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Unit tests for dc_common::BatteryCycleAccumulator (#360). No node, no rclcpp::init(): charge
// percentage, power-supply status and the timestamp all arrive as arguments, so a week of
// charge/discharge runs in microseconds.

#include "dc_common/battery_cycle_accumulator.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>

using dc_common::BatteryCycleAccumulator;
using dc_common::PowerSupplyStatus;

namespace
{

std::chrono::system_clock::time_point at(int seconds)
{
  return std::chrono::system_clock::time_point(std::chrono::seconds(seconds));
}

}  // namespace

TEST(BatteryCycleAccumulator, StartsIdle)
{
  BatteryCycleAccumulator accumulator;
  EXPECT_FALSE(accumulator.charging());
  EXPECT_FALSE(accumulator.openSession().has_value());
  EXPECT_FALSE(accumulator.lastPercentage().has_value());
  EXPECT_DOUBLE_EQ(accumulator.accumulatedDischarge(), 0.0);
  EXPECT_EQ(accumulator.completedCycles(), 0u);
}

TEST(BatteryCycleAccumulator, TwoPartialDischargesSummingToAFullOneAreOneCycle)
{
  BatteryCycleAccumulator accumulator;
  accumulator.update(100.0, PowerSupplyStatus::Discharging, at(0));
  accumulator.update(50.0, PowerSupplyStatus::Discharging, at(3600));
  accumulator.update(100.0, PowerSupplyStatus::Charging, at(7200));

  // Half a pack gone, recharged: no cycle yet, however many charges have happened.
  EXPECT_DOUBLE_EQ(accumulator.accumulatedDischarge(), 50.0);
  EXPECT_EQ(accumulator.completedCycles(), 0u);

  accumulator.update(50.0, PowerSupplyStatus::Discharging, at(10800));
  EXPECT_DOUBLE_EQ(accumulator.accumulatedDischarge(), 100.0);
  EXPECT_EQ(accumulator.completedCycles(), 1u);

  // And the naive count would already be at three charges by here, still one cycle.
  accumulator.update(100.0, PowerSupplyStatus::Charging, at(14400));
  accumulator.update(75.0, PowerSupplyStatus::Discharging, at(18000));
  EXPECT_EQ(accumulator.completedCycles(), 1u);
}

TEST(BatteryCycleAccumulator, ChargingIsNotDischarge)
{
  BatteryCycleAccumulator accumulator;
  accumulator.update(20.0, PowerSupplyStatus::Charging, at(0));
  accumulator.update(60.0, PowerSupplyStatus::Charging, at(1800));
  accumulator.update(100.0, PowerSupplyStatus::Full, at(3600));

  EXPECT_DOUBLE_EQ(accumulator.accumulatedDischarge(), 0.0);
  EXPECT_EQ(accumulator.completedCycles(), 0u);
}

TEST(BatteryCycleAccumulator, SessionsAreDelimitedByStatusNotByPercentage)
{
  // A percentage bouncing across any plausible threshold, with the pack never on a charger: not
  // one session opens.
  BatteryCycleAccumulator accumulator;
  const double noisy[] = { 79.0, 81.0, 78.0, 82.0, 77.0, 83.0, 20.0, 21.0, 19.0, 22.0 };
  int second = 0;
  for (const double percentage : noisy)
  {
    const auto update = accumulator.update(percentage, PowerSupplyStatus::Discharging, at(second));
    EXPECT_FALSE(update.started.has_value()) << "at second " << second;
    EXPECT_FALSE(update.ended.has_value()) << "at second " << second;
    second += 60;
  }
  EXPECT_FALSE(accumulator.charging());

  // And a percentage that is falling while the charger says Charging still opens exactly one.
  const auto opened = accumulator.update(21.0, PowerSupplyStatus::Charging, at(second));
  EXPECT_TRUE(opened.started.has_value());
  for (int i = 0; i < 5; ++i)
  {
    second += 60;
    const auto update = accumulator.update(20.0 - i, PowerSupplyStatus::Charging, at(second));
    EXPECT_FALSE(update.started.has_value());
    EXPECT_FALSE(update.ended.has_value());
  }
  EXPECT_TRUE(accumulator.charging());
}

TEST(BatteryCycleAccumulator, SessionReportsItsBoundariesAndThePrecedingDischargeDepth)
{
  BatteryCycleAccumulator accumulator;
  accumulator.update(95.0, PowerSupplyStatus::Discharging, at(0));
  accumulator.update(33.0, PowerSupplyStatus::Discharging, at(6000));

  const auto opened = accumulator.update(33.0, PowerSupplyStatus::Charging, at(6060));
  ASSERT_TRUE(opened.started.has_value());
  const auto& started = *opened.started;
  EXPECT_EQ(started.sequence, 1u);
  EXPECT_EQ(started.started, at(6060));
  EXPECT_TRUE(started.open());
  ASSERT_TRUE(started.start_percentage.has_value());
  EXPECT_DOUBLE_EQ(*started.start_percentage, 33.0);
  EXPECT_DOUBLE_EQ(started.preceding_discharge_depth, 62.0);

  const auto closed = accumulator.update(100.0, PowerSupplyStatus::Full, at(12060));
  ASSERT_TRUE(closed.ended.has_value());
  const auto& ended = *closed.ended;
  EXPECT_EQ(ended.sequence, 1u);
  EXPECT_FALSE(ended.open());
  EXPECT_EQ(ended.started, at(6060));
  ASSERT_TRUE(ended.ended.has_value());
  EXPECT_EQ(*ended.ended, at(12060));
  ASSERT_TRUE(ended.end_percentage.has_value());
  EXPECT_DOUBLE_EQ(*ended.end_percentage, 100.0);
  EXPECT_DOUBLE_EQ(ended.preceding_discharge_depth, 62.0);
  EXPECT_EQ(ended.duration(at(99999)), std::chrono::seconds(6000));
  EXPECT_FALSE(accumulator.charging());
}

TEST(BatteryCycleAccumulator, EachSessionReportsOnlyTheDischargeThatPrecededIt)
{
  BatteryCycleAccumulator accumulator;
  accumulator.update(100.0, PowerSupplyStatus::Discharging, at(0));
  accumulator.update(70.0, PowerSupplyStatus::Discharging, at(1000));
  const auto first = accumulator.update(70.0, PowerSupplyStatus::Charging, at(1100));
  accumulator.update(100.0, PowerSupplyStatus::Full, at(2000));

  accumulator.update(45.0, PowerSupplyStatus::Discharging, at(5000));
  const auto second = accumulator.update(45.0, PowerSupplyStatus::Charging, at(5100));

  ASSERT_TRUE(first.started.has_value());
  ASSERT_TRUE(second.started.has_value());
  EXPECT_DOUBLE_EQ(first.started->preceding_discharge_depth, 30.0);
  EXPECT_DOUBLE_EQ(second.started->preceding_discharge_depth, 55.0);
  EXPECT_EQ(second.started->sequence, 2u);
}

TEST(BatteryCycleAccumulator, SessionStillChargingIsReportedOpenWithTheTimeItHasRun)
{
  BatteryCycleAccumulator accumulator;
  accumulator.update(40.0, PowerSupplyStatus::Charging, at(600));

  ASSERT_TRUE(accumulator.openSession().has_value());
  const auto& open = *accumulator.openSession();
  EXPECT_TRUE(open.open());
  EXPECT_FALSE(open.ended.has_value());
  EXPECT_EQ(open.duration(at(3000)), std::chrono::seconds(2400));
  EXPECT_NE(open.duration(at(3000)), std::chrono::seconds(0));
}

TEST(BatteryCycleAccumulator, AnUnknownStatusNeitherOpensNorClosesASession)
{
  // A momentary status dropout mid-charge is one session, not two.
  BatteryCycleAccumulator accumulator;
  const auto opened = accumulator.update(30.0, PowerSupplyStatus::Charging, at(0));
  ASSERT_TRUE(opened.started.has_value());

  const auto dropout = accumulator.update(35.0, PowerSupplyStatus::Unknown, at(300));
  EXPECT_FALSE(dropout.started.has_value());
  EXPECT_FALSE(dropout.ended.has_value());
  EXPECT_TRUE(accumulator.charging());

  const auto resumed = accumulator.update(40.0, PowerSupplyStatus::Charging, at(600));
  EXPECT_FALSE(resumed.started.has_value());

  const auto closed = accumulator.update(100.0, PowerSupplyStatus::Full, at(1200));
  ASSERT_TRUE(closed.ended.has_value());
  EXPECT_EQ(closed.ended->sequence, 1u);
  EXPECT_EQ(closed.ended->duration(at(1200)), std::chrono::seconds(1200));
}

TEST(BatteryCycleAccumulator, UnknownStatusStillAccumulatesDischarge)
{
  BatteryCycleAccumulator accumulator;
  accumulator.update(90.0, PowerSupplyStatus::Unknown, at(0));
  accumulator.update(60.0, PowerSupplyStatus::Unknown, at(600));

  EXPECT_DOUBLE_EQ(accumulator.accumulatedDischarge(), 30.0);
  EXPECT_FALSE(accumulator.charging());
}

TEST(BatteryCycleAccumulator, NotChargingAndFullBothCloseASession)
{
  for (const auto status : { PowerSupplyStatus::NotCharging, PowerSupplyStatus::Full, PowerSupplyStatus::Discharging })
  {
    BatteryCycleAccumulator accumulator;
    accumulator.update(50.0, PowerSupplyStatus::Charging, at(0));
    const auto closed = accumulator.update(90.0, status, at(60));
    EXPECT_TRUE(closed.ended.has_value()) << "status " << static_cast<int>(status);
    EXPECT_FALSE(accumulator.charging());
  }
}

TEST(BatteryCycleAccumulator, ASampleWithNoPercentageStillDelimitsSessions)
{
  // Hardware that reports a status but no charge percentage: the session is still recorded, its
  // boundary percentages simply absent rather than filled with a made-up zero.
  BatteryCycleAccumulator accumulator;
  const auto opened = accumulator.update(std::nullopt, PowerSupplyStatus::Charging, at(0));
  ASSERT_TRUE(opened.started.has_value());
  EXPECT_FALSE(opened.started->start_percentage.has_value());

  const auto closed = accumulator.update(std::nullopt, PowerSupplyStatus::Discharging, at(900));
  ASSERT_TRUE(closed.ended.has_value());
  EXPECT_FALSE(closed.ended->end_percentage.has_value());
  EXPECT_DOUBLE_EQ(closed.ended->preceding_discharge_depth, 0.0);
  EXPECT_EQ(closed.ended->duration(at(900)), std::chrono::seconds(900));
}

TEST(BatteryCycleAccumulator, ASampleWithNoPercentageKeepsTheLastKnownOne)
{
  BatteryCycleAccumulator accumulator;
  accumulator.update(64.0, PowerSupplyStatus::Discharging, at(0));
  accumulator.update(std::nullopt, PowerSupplyStatus::Discharging, at(60));

  ASSERT_TRUE(accumulator.lastPercentage().has_value());
  EXPECT_DOUBLE_EQ(*accumulator.lastPercentage(), 64.0);

  const auto opened = accumulator.update(std::nullopt, PowerSupplyStatus::Charging, at(120));
  ASSERT_TRUE(opened.started.has_value());
  ASSERT_TRUE(opened.started->start_percentage.has_value());
  EXPECT_DOUBLE_EQ(*opened.started->start_percentage, 64.0);
}

TEST(BatteryCycleAccumulator, CountsCyclesAcrossManySmallTopUps)
{
  // The opportunity-charging pattern a fleet robot actually runs: ten 10-point discharges, each
  // topped straight back up. One cycle, not ten.
  BatteryCycleAccumulator accumulator;
  accumulator.update(100.0, PowerSupplyStatus::Discharging, at(0));
  int second = 0;
  for (int i = 0; i < 10; ++i)
  {
    second += 600;
    accumulator.update(90.0, PowerSupplyStatus::Discharging, at(second));
    second += 60;
    accumulator.update(90.0, PowerSupplyStatus::Charging, at(second));
    second += 300;
    accumulator.update(100.0, PowerSupplyStatus::Full, at(second));
  }

  EXPECT_DOUBLE_EQ(accumulator.accumulatedDischarge(), 100.0);
  EXPECT_EQ(accumulator.completedCycles(), 1u);
}
