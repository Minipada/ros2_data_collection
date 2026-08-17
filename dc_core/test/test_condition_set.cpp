// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Unit tests for dc_core::ConditionSet, the if_all/if_any/if_none composition extracted from
// dc_measurements::Measurement::isConditionOn() (#284).
//
// The evaluator is ROS-independent by design -- the EdgeTrigger plugin (#282) needs the same
// composition without a node -- so these tests never call rclcpp::init() and never build a node.

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include "dc_core/condition_set.hpp"

namespace
{

struct EvaluationCase
{
  std::string name;
  std::vector<std::string> if_all;
  std::vector<std::string> if_any;
  std::vector<std::string> if_none;
  dc_core::ConditionStates states;
  bool expected;
};

class ConditionSetEvaluationTest : public ::testing::TestWithParam<EvaluationCase>
{
};

// One table over the whole truth space of the three groups: satisfied, unsatisfied, and named
// but missing from the state map, in isolation and in combination.
const std::vector<EvaluationCase> kEvaluationCases = {
  // Nothing configured: the set imposes no objection at all.
  { "EmptySetIsSatisfied", {}, {}, {}, {}, true },
  { "EmptySetIgnoresUnrelatedStates", {}, {}, {}, { { "unrelated", false } }, true },

  // if_all: every named Condition must be true.
  { "IfAllSingleTrue", { "a" }, {}, {}, { { "a", true } }, true },
  { "IfAllSingleFalse", { "a" }, {}, {}, { { "a", false } }, false },
  { "IfAllEveryNameTrue", { "a", "b", "c" }, {}, {}, { { "a", true }, { "b", true }, { "c", true } }, true },
  { "IfAllOneNameFalse", { "a", "b", "c" }, {}, {}, { { "a", true }, { "b", false }, { "c", true } }, false },
  { "IfAllLastNameFalse", { "a", "b" }, {}, {}, { { "a", true }, { "b", false } }, false },
  { "IfAllMissingNameBlocks", { "a", "missing" }, {}, {}, { { "a", true } }, false },

  // if_any: at least one named Condition must be true.
  { "IfAnySingleTrue", {}, { "a" }, {}, { { "a", true } }, true },
  { "IfAnySingleFalse", {}, { "a" }, {}, { { "a", false } }, false },
  { "IfAnyFirstNameTrue", {}, { "a", "b" }, {}, { { "a", true }, { "b", false } }, true },
  { "IfAnyLastNameTrue", {}, { "a", "b" }, {}, { { "a", false }, { "b", true } }, true },
  { "IfAnyEveryNameFalse", {}, { "a", "b" }, {}, { { "a", false }, { "b", false } }, false },
  { "IfAnyMissingNameDoesNotSatisfy", {}, { "missing" }, {}, {}, false },
  { "IfAnyMissingNameAlongsideTrueOne", {}, { "missing", "a" }, {}, { { "a", true } }, true },

  // if_none: every named Condition must be false.
  { "IfNoneSingleFalse", {}, {}, { "a" }, { { "a", false } }, true },
  { "IfNoneSingleTrue", {}, {}, { "a" }, { { "a", true } }, false },
  { "IfNoneEveryNameFalse", {}, {}, { "a", "b" }, { { "a", false }, { "b", false } }, true },
  { "IfNoneOneNameTrue", {}, {}, { "a", "b" }, { { "a", false }, { "b", true } }, false },
  // A name with no known state reads as false, which is exactly what if_none asks for.
  { "IfNoneMissingNameIsSatisfied", {}, {}, { "missing" }, {}, true },

  // Combinations: the three groups are ANDed, so any one of them can block the set.
  { "AllGroupsSatisfied",
    { "a" },
    { "b", "c" },
    { "d" },
    { { "a", true }, { "b", false }, { "c", true }, { "d", false } },
    true },
  { "IfAllBlocksOtherwiseSatisfiedSet",
    { "a" },
    { "b" },
    { "c" },
    { { "a", false }, { "b", true }, { "c", false } },
    false },
  { "IfAnyBlocksOtherwiseSatisfiedSet",
    { "a" },
    { "b" },
    { "c" },
    { { "a", true }, { "b", false }, { "c", false } },
    false },
  { "IfNoneBlocksOtherwiseSatisfiedSet",
    { "a" },
    { "b" },
    { "c" },
    { { "a", true }, { "b", true }, { "c", true } },
    false },

  // A name may appear in several groups; it must then satisfy each of them.
  { "SameNameInIfAllAndIfAnyWhenTrue", { "a" }, { "a" }, {}, { { "a", true } }, true },
  { "SameNameInIfAllAndIfNoneIsUnsatisfiable", { "a" }, {}, { "a" }, { { "a", true } }, false },
  { "SameNameInIfAllAndIfNoneIsUnsatisfiableWhenFalseToo", { "a" }, {}, { "a" }, { { "a", false } }, false },
};

TEST_P(ConditionSetEvaluationTest, EvaluatesToExpectedResult)
{
  const auto& test_case = GetParam();
  const dc_core::ConditionSet condition_set(test_case.if_all, test_case.if_any, test_case.if_none);

  EXPECT_EQ(condition_set.isSatisfied(test_case.states), test_case.expected);
  EXPECT_EQ(condition_set.evaluate(test_case.states).satisfied(), test_case.expected);
}

// The lookup overload answers identically to the map overload when the lookup reads that same
// map -- Measurement uses the lookup, a caller holding pre-collected states uses the map.
TEST_P(ConditionSetEvaluationTest, LookupOverloadAgreesWithMapOverload)
{
  const auto& test_case = GetParam();
  const dc_core::ConditionSet condition_set(test_case.if_all, test_case.if_any, test_case.if_none);

  const dc_core::ConditionStateLookup lookup = [&test_case](const std::string& name) {
    const auto state = test_case.states.find(name);
    return state != test_case.states.end() && state->second;
  };

  EXPECT_EQ(condition_set.isSatisfied(lookup), test_case.expected);
}

INSTANTIATE_TEST_SUITE_P(ConditionSet, ConditionSetEvaluationTest, ::testing::ValuesIn(kEvaluationCases),
                         [](const ::testing::TestParamInfo<EvaluationCase>& info) { return info.param.name; });

TEST(ConditionSetTest, DefaultConstructedSetIsEmptyAndSatisfied)
{
  const dc_core::ConditionSet condition_set;

  EXPECT_TRUE(condition_set.empty());
  EXPECT_TRUE(condition_set.isSatisfied(dc_core::ConditionStates{}));
}

TEST(ConditionSetTest, IsNotEmptyWhenAnySingleGroupIsPopulated)
{
  EXPECT_FALSE(dc_core::ConditionSet({ "a" }, {}, {}).empty());
  EXPECT_FALSE(dc_core::ConditionSet({}, { "a" }, {}).empty());
  EXPECT_FALSE(dc_core::ConditionSet({}, {}, { "a" }).empty());
  EXPECT_TRUE(dc_core::ConditionSet({}, {}, {}).empty());
}

TEST(ConditionSetTest, KeepsTheNamesItWasBuiltWith)
{
  const dc_core::ConditionSet condition_set({ "a" }, { "b", "c" }, { "d" });

  EXPECT_EQ(condition_set.ifAll(), std::vector<std::string>({ "a" }));
  EXPECT_EQ(condition_set.ifAny(), std::vector<std::string>({ "b", "c" }));
  EXPECT_EQ(condition_set.ifNone(), std::vector<std::string>({ "d" }));
}

TEST(ConditionSetTest, NamesListsEveryConditionOnceInDeclarationOrder)
{
  const dc_core::ConditionSet condition_set({ "a", "b" }, { "b", "c" }, { "a", "d" });

  EXPECT_EQ(condition_set.names(), std::vector<std::string>({ "a", "b", "c", "d" }));
  EXPECT_TRUE(dc_core::ConditionSet().names().empty());
}

// The per-group breakdown is what Measurement logs at debug level; each group answers for itself,
// and an empty group never objects.
TEST(ConditionSetTest, OutcomeReportsWhichGroupObjected)
{
  const dc_core::ConditionSet condition_set({ "a" }, { "b" }, { "c" });

  const auto outcome = condition_set.evaluate(dc_core::ConditionStates{
      { "a", false },
      { "b", true },
      { "c", true },
  });

  EXPECT_FALSE(outcome.if_all_satisfied);
  EXPECT_TRUE(outcome.if_any_satisfied);
  EXPECT_FALSE(outcome.if_none_satisfied);
  EXPECT_FALSE(outcome.satisfied());

  const auto empty_outcome = dc_core::ConditionSet().evaluate(dc_core::ConditionStates{});
  EXPECT_TRUE(empty_outcome.if_all_satisfied);
  EXPECT_TRUE(empty_outcome.if_any_satisfied);
  EXPECT_TRUE(empty_outcome.if_none_satisfied);
  EXPECT_TRUE(empty_outcome.satisfied());
}

// Which Conditions get polled is observable behavior, not an implementation detail:
// dc_conditions::Condition::getState() publishes on /dc/condition/<name>, and stateful Conditions
// such as SameAsPrevious advance their state when polled. Each group stops at the name that
// settles it...
TEST(ConditionSetTest, EachGroupStopsQueryingOnceItsAnswerIsSettled)
{
  std::vector<std::string> queried;
  const dc_core::ConditionStateLookup lookup = [&queried](const std::string& name) {
    queried.push_back(name);
    return name == "true_condition";
  };

  dc_core::ConditionSet({ "false_condition", "unreached" }, {}, {}).isSatisfied(lookup);
  EXPECT_EQ(queried, std::vector<std::string>({ "false_condition" }));

  queried.clear();
  dc_core::ConditionSet({}, { "true_condition", "unreached" }, {}).isSatisfied(lookup);
  EXPECT_EQ(queried, std::vector<std::string>({ "true_condition" }));

  queried.clear();
  dc_core::ConditionSet({}, {}, { "true_condition", "unreached" }).isSatisfied(lookup);
  EXPECT_EQ(queried, std::vector<std::string>({ "true_condition" }));
}

// ...but a group that already objects does not stop the following groups from being evaluated,
// which is how isConditionOn() behaved before the extraction.
TEST(ConditionSetTest, EveryGroupIsEvaluatedEvenAfterAnEarlierOneObjects)
{
  std::vector<std::string> queried;
  const dc_core::ConditionStateLookup lookup = [&queried](const std::string& name) {
    queried.push_back(name);
    return false;
  };

  EXPECT_FALSE(dc_core::ConditionSet({ "a" }, { "b" }, { "c" }).isSatisfied(lookup));
  EXPECT_EQ(queried, std::vector<std::string>({ "a", "b", "c" }));
}

}  // namespace
