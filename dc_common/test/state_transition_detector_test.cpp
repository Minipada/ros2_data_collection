// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Unit tests for dc_common::StateTransitionDetector (#360). No node, no rclcpp::init(): every
// timestamp is passed in, so a whole shift's worth of transitions runs in microseconds.

#include "dc_common/state_transition_detector.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

using dc_common::StateTransitionDetector;

namespace
{

std::chrono::system_clock::time_point at(int seconds)
{
  return std::chrono::system_clock::time_point(std::chrono::seconds(seconds));
}

/// The closed mode set driving_type emits, standing in for the intervention Measurement's input.
enum class Mode
{
  Autonomous,
  Manual,
  Teleop,
  Unknown,
};

}  // namespace

TEST(StateTransitionDetector, ReportsNoStateBeforeAnySample)
{
  StateTransitionDetector<Mode> detector;
  EXPECT_FALSE(detector.state().has_value());
  EXPECT_FALSE(detector.openInterval(at(10)).has_value());
  EXPECT_EQ(detector.sequence(), 0u);
}

TEST(StateTransitionDetector, FirstSampleIsNotATransition)
{
  // Nothing was left, so there is no previous state or dwell to report -- but the state is held.
  StateTransitionDetector<Mode> detector;
  EXPECT_FALSE(detector.update(Mode::Autonomous, at(0)).has_value());
  ASSERT_TRUE(detector.state().has_value());
  EXPECT_EQ(*detector.state(), Mode::Autonomous);
  EXPECT_EQ(detector.sequence(), 0u);
}

TEST(StateTransitionDetector, RepeatedIdenticalStateNeverTransitions)
{
  StateTransitionDetector<Mode> detector;
  detector.update(Mode::Autonomous, at(0));
  for (int second = 1; second <= 1000; ++second)
  {
    EXPECT_FALSE(detector.update(Mode::Autonomous, at(second)).has_value()) << "at second " << second;
  }
  EXPECT_EQ(detector.sequence(), 0u);

  // The dwell of that unbroken run is still counted from the first sample, not the last.
  const auto transition = detector.update(Mode::Manual, at(1001));
  ASSERT_TRUE(transition.has_value());
  EXPECT_EQ(transition->dwell, std::chrono::seconds(1001));
}

TEST(StateTransitionDetector, TransitionReportsPreviousStateAndItsDwell)
{
  StateTransitionDetector<Mode> detector;
  detector.update(Mode::Autonomous, at(0));

  const auto transition = detector.update(Mode::Manual, at(120));
  ASSERT_TRUE(transition.has_value());
  EXPECT_EQ(transition->from, Mode::Autonomous);
  EXPECT_EQ(transition->to, Mode::Manual);
  EXPECT_EQ(transition->at, at(120));
  EXPECT_EQ(transition->dwell, std::chrono::seconds(120));
  EXPECT_EQ(transition->sequence, 1u);
}

TEST(StateTransitionDetector, SequenceStaysMonotonicThroughARapidBurst)
{
  StateTransitionDetector<Mode> detector;
  detector.update(Mode::Autonomous, at(0));

  // Alternating modes at the same timestamp: a burst faster than the clock's resolution must still
  // number every transition, otherwise a consumer cannot tell a dropped Record from a quiet period.
  std::vector<std::uint64_t> sequences;
  for (int i = 0; i < 50; ++i)
  {
    const auto transition = detector.update(i % 2 == 0 ? Mode::Manual : Mode::Autonomous, at(0));
    ASSERT_TRUE(transition.has_value()) << "at burst step " << i;
    EXPECT_EQ(transition->dwell, std::chrono::seconds(0));
    sequences.push_back(transition->sequence);
  }

  for (std::size_t i = 0; i < sequences.size(); ++i)
  {
    EXPECT_EQ(sequences[i], i + 1);
  }
  EXPECT_EQ(detector.sequence(), 50u);
}

TEST(StateTransitionDetector, IntervalStillOpenIsReportedOpenWithTheTimeItHasRun)
{
  // The shutdown case: collection stops mid-intervention. The interval comes back through
  // openInterval(), never as a transition, so it can't be averaged in as a zero-length one.
  StateTransitionDetector<Mode> detector;
  detector.update(Mode::Autonomous, at(0));
  detector.update(Mode::Manual, at(60));

  const auto open = detector.openInterval(at(300));
  ASSERT_TRUE(open.has_value());
  EXPECT_EQ(open->state, Mode::Manual);
  EXPECT_EQ(open->entered, at(60));
  EXPECT_EQ(open->elapsed, std::chrono::seconds(240));
  EXPECT_NE(open->elapsed, std::chrono::seconds(0));
  EXPECT_EQ(open->sequence, 1u);
}

TEST(StateTransitionDetector, OpenIntervalOfTheFirstStateCarriesNoTransitionSequence)
{
  StateTransitionDetector<Mode> detector;
  detector.update(Mode::Autonomous, at(5));

  const auto open = detector.openInterval(at(35));
  ASSERT_TRUE(open.has_value());
  EXPECT_EQ(open->state, Mode::Autonomous);
  EXPECT_EQ(open->elapsed, std::chrono::seconds(30));
  EXPECT_EQ(open->sequence, 0u);
}

TEST(StateTransitionDetector, OpenIntervalDoesNotConsumeTheState)
{
  StateTransitionDetector<Mode> detector;
  detector.update(Mode::Teleop, at(0));

  EXPECT_EQ(detector.openInterval(at(10))->elapsed, std::chrono::seconds(10));
  EXPECT_EQ(detector.openInterval(at(20))->elapsed, std::chrono::seconds(20));

  const auto transition = detector.update(Mode::Autonomous, at(30));
  ASSERT_TRUE(transition.has_value());
  EXPECT_EQ(transition->dwell, std::chrono::seconds(30));
}

TEST(StateTransitionDetector, SeededStateDwellsFromTheSeedTimestamp)
{
  StateTransitionDetector<Mode> detector(Mode::Autonomous, at(100));

  const auto transition = detector.update(Mode::Manual, at(160));
  ASSERT_TRUE(transition.has_value());
  EXPECT_EQ(transition->from, Mode::Autonomous);
  EXPECT_EQ(transition->dwell, std::chrono::seconds(60));
}

TEST(StateTransitionDetector, OutOfOrderSampleDoesNotReportANegativeDwell)
{
  StateTransitionDetector<Mode> detector;
  detector.update(Mode::Autonomous, at(100));

  const auto transition = detector.update(Mode::Manual, at(40));
  ASSERT_TRUE(transition.has_value());
  EXPECT_EQ(transition->dwell, std::chrono::seconds(0));
  EXPECT_EQ(detector.openInterval(at(0))->elapsed, std::chrono::seconds(0));
}

TEST(StateTransitionDetector, WorksOverStringStatesToo)
{
  // driving_type's mode strings and a mission's outcome enum are meant to fit the same detector.
  StateTransitionDetector<std::string> detector;
  detector.update("autonomous", at(0));
  EXPECT_FALSE(detector.update("autonomous", at(5)).has_value());

  const auto transition = detector.update("teleop", at(45));
  ASSERT_TRUE(transition.has_value());
  EXPECT_EQ(transition->from, "autonomous");
  EXPECT_EQ(transition->to, "teleop");
  EXPECT_EQ(transition->dwell, std::chrono::seconds(45));
  EXPECT_EQ(transition->sequence, 1u);
}
