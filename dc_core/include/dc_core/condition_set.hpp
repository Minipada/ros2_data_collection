// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_CORE_CONDITION_SET_HPP_
#define DC_CORE_CONDITION_SET_HPP_

#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace dc_core
{

/**
 * @brief State of every Condition an evaluation may consult, keyed by Condition name.
 *
 * A name absent from the map reads as false -- see ConditionSet::evaluate().
 */
using ConditionStates = std::map<std::string, bool>;

/**
 * @brief Pull-based equivalent of ConditionStates: returns the current state of one Condition.
 *
 * Callers that own live dc_core::Condition plugins pass a lookup instead of a pre-built map when
 * evaluating a Condition has a cost or a side effect (dc_conditions::Condition::getState()
 * publishes on /dc/condition/<name>, and stateful Conditions such as SameAsPrevious update their
 * internal state when polled). With a lookup, only the Conditions the boolean composition
 * actually needs are queried.
 */
using ConditionStateLookup = std::function<bool(const std::string& condition_name)>;

/**
 * @brief Per-group result of evaluating a ConditionSet, for logging and debugging.
 *
 * Each member is true when its group imposes no objection -- which includes the case of an empty
 * group. satisfied() is the single answer the ConditionSet yields.
 */
struct ConditionSetOutcome
{
  bool if_all_satisfied{ true };
  bool if_any_satisfied{ true };
  bool if_none_satisfied{ true };

  bool satisfied() const
  {
    return if_all_satisfied && if_any_satisfied && if_none_satisfied;
  }
};

/**
 * @class dc_core::ConditionSet
 * @brief The if_all/if_any/if_none boolean composition of Conditions, without any ROS dependency.
 *
 * A ConditionSet holds three lists of Condition names and answers one question: given the current
 * state of those Conditions, is the set satisfied?
 *
 *   - if_all:  every named Condition must be true  (empty list: no objection)
 *   - if_any:  at least one named Condition must be true  (empty list: no objection)
 *   - if_none: every named Condition must be false  (empty list: no objection)
 *
 * The three groups are ANDed together, so an empty ConditionSet is satisfied. Names appearing in
 * more than one group are evaluated per group and must satisfy each of them.
 *
 * A name with no known state -- absent from the ConditionStates map, or reported false by a
 * ConditionStateLookup that cannot resolve it -- reads as false. That makes an unresolvable name
 * in if_all or if_any block the set, and satisfy if_none.
 *
 * Owned by dc_core, next to the Condition base class, because more than one plugin composes
 * Conditions this way: dc_measurements::Measurement gates collection on it, and the EdgeTrigger
 * plugin (#282) needs the same composition without duplicating it.
 */
class ConditionSet
{
public:
  ConditionSet() = default;

  ConditionSet(std::vector<std::string> if_all, std::vector<std::string> if_any, std::vector<std::string> if_none)
    : if_all_{ std::move(if_all) }, if_any_{ std::move(if_any) }, if_none_{ std::move(if_none) }
  {
  }

  /**
   * @brief Whether no Condition at all is named, in which case the set is always satisfied.
   */
  bool empty() const
  {
    return if_all_.empty() && if_any_.empty() && if_none_.empty();
  }

  const std::vector<std::string>& ifAll() const
  {
    return if_all_;
  }

  const std::vector<std::string>& ifAny() const
  {
    return if_any_;
  }

  const std::vector<std::string>& ifNone() const
  {
    return if_none_;
  }

  /**
   * @brief Every Condition name this set may consult, each listed once, in if_all/if_any/if_none
   * declaration order.
   *
   * Meant for callers that build a ConditionStates map up front and need to know what to put in
   * it, and for configuration-time validation of the names against the configured Conditions.
   */
  std::vector<std::string> names() const
  {
    std::vector<std::string> all_names;
    all_names.reserve(if_all_.size() + if_any_.size() + if_none_.size());
    for (const auto* group : { &if_all_, &if_any_, &if_none_ })
    {
      for (const auto& name : *group)
      {
        if (std::find(all_names.begin(), all_names.end(), name) == all_names.end())
        {
          all_names.push_back(name);
        }
      }
    }
    return all_names;
  }

  /**
   * @brief Evaluate the set against a lookup of Condition states.
   *
   * Each group stops querying as soon as its own answer is settled (if_all on the first false,
   * if_any on the first true, if_none on the first true), but all three groups are always
   * evaluated, even when an earlier one already objects -- Conditions have side effects, so which
   * ones get polled is part of the observable behavior and is kept as it was.
   */
  ConditionSetOutcome evaluate(const ConditionStateLookup& state_of) const
  {
    ConditionSetOutcome outcome;

    if (!if_all_.empty())
    {
      outcome.if_all_satisfied =
          std::all_of(if_all_.begin(), if_all_.end(), [&](const std::string& name) { return state_of(name); });
    }

    if (!if_any_.empty())
    {
      outcome.if_any_satisfied =
          std::any_of(if_any_.begin(), if_any_.end(), [&](const std::string& name) { return state_of(name); });
    }

    if (!if_none_.empty())
    {
      outcome.if_none_satisfied =
          std::all_of(if_none_.begin(), if_none_.end(), [&](const std::string& name) { return !state_of(name); });
    }

    return outcome;
  }

  /**
   * @brief Evaluate the set against a map of Condition states. Names absent from the map read as
   * false.
   */
  ConditionSetOutcome evaluate(const ConditionStates& states) const
  {
    return evaluate(ConditionStateLookup{ [&states](const std::string& name) {
      const auto state = states.find(name);
      return state != states.end() && state->second;
    } });
  }

  /**
   * @brief evaluate(), reduced to the single boolean most callers want.
   */
  bool isSatisfied(const ConditionStateLookup& state_of) const
  {
    return evaluate(state_of).satisfied();
  }

  bool isSatisfied(const ConditionStates& states) const
  {
    return evaluate(states).satisfied();
  }

private:
  std::vector<std::string> if_all_;
  std::vector<std::string> if_any_;
  std::vector<std::string> if_none_;
};

}  // namespace dc_core

#endif  // DC_CORE_CONDITION_SET_HPP_
