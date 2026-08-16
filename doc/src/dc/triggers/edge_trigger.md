# Edge trigger

## Description

Composes the Condition plugins named in `if_all_conditions`/`if_any_conditions`/
`if_none_conditions` — the exact same rules Measurements already use — and fires exactly
once on the false→true rising edge of the composed result. A sustained `true` fires only
once; the Trigger stays quiet until the composed result falls back to `false` and rises
again. No rule-evaluation logic of its own: composition is entirely delegated to the same
`ConditionSet` evaluator `dc_measurements::Measurement` uses for live gating.

## Parameters

See [Node parameters](../triggers.md#node-parameters) — `EdgeTrigger` has no parameters
of its own beyond the shared `trigger.*` ones every Trigger plugin reads.
