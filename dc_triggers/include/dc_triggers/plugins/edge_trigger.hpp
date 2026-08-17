// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#ifndef DC_TRIGGERS__PLUGINS__EDGE_TRIGGER_HPP_
#define DC_TRIGGERS__PLUGINS__EDGE_TRIGGER_HPP_

#include "dc_triggers/trigger.hpp"

namespace dc_triggers
{

/**
 * @class dc_triggers::EdgeTrigger
 * @brief Fires exactly once on the false->true rising edge of its ConditionSet's composed result.
 * No bespoke rule-evaluation logic of its own -- if_all/if_any/if_none composition is entirely
 * delegated to dc_core::ConditionSet, the same evaluator dc_measurements::Measurement uses for
 * live gating (#284), so the two mechanisms can't silently diverge in how they interpret the same
 * rule syntax.
 */
class EdgeTrigger : public dc_triggers::Trigger
{
public:
  EdgeTrigger();
  ~EdgeTrigger() override;

  bool checkFired() override;

private:
  // Whether the composed ConditionSet was satisfied on the previous checkFired() call, so a
  // sustained true only fires once, on the transition into it.
  bool previously_satisfied_{ false };
};

}  // namespace dc_triggers

#endif  // DC_TRIGGERS__PLUGINS__EDGE_TRIGGER_HPP_
