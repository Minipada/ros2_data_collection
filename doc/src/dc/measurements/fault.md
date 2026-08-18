# Fault

## Description

Reports the transitions `diagnostics`'s periodic snapshots leave a consumer to reconstruct:
`diagnostics` writes the current `DiagnosticStatus` level on every poll, so recovering "when did
this component break, and when was it fixed" means scanning for where a value changed in
SQL -- exactly the failure mode transition Records exist to prevent. Fault subscribes to the same
`/diagnostics` topic (`diagnostic_msgs/DiagnosticArray`) and emits one Record per component **only
when its level actually changes**, feeding each watched component's stream of `(level, timestamp)`
samples to its own `dc_common::StateTransitionDetector`, a header-only `dc_common` type generic
over the state being tracked and owning no clock of its own.

Every Record carries the component, the level it left (`from_level`) and entered (`to_level`), how
long the previous level had been held (`previous_level_duration_s`), and the status message as the
reason. Levels are the `DiagnosticStatus` constants `OK`, `WARN`, `ERROR` and `STALE` -- reported
by name, so a silent (`STALE`) component is never confused with a broken (`ERROR`) one.

Each transition is also classified as an `event`:

- **raise** -- the component left `OK`. A new fault opens (`"state": "open"`), and
  `fault_started_at` records when.
- **change** -- a transition between two non-`OK` levels (e.g. `WARN` to `ERROR`). The fault stays
  open; `fault_started_at` is unchanged from the raise that opened it.
- **clear** -- the component returned to `OK`. The fault closes (`"state": "closed"`) and the
  Record carries `duration_s`, the length of the whole fault, alongside the same `fault_started_at`
  the raise carried -- so the pair can be joined downstream for MTTR. MTBF is the gap between
  successive `raise` events for a component.

A fault still open when collection stops keeps its last Record's `"state": "open"` and no
`duration_s`: no closing Record is invented, so it cannot be read as a zero-length outage. Every
Record carries a `seq` that increments by one across every watched component, so a dropped Record
is a detectable gap rather than a silently shortened outage. A component observed already faulted
on its very first sample has no `fault_started_at` on that Record -- the detector treats the first
sample as a baseline, not a transition, since no start was ever seen.

## Parameters

| Parameter | Description                                                                             | Type      | Default            |
| --------- | ---------------------------------------------------------------------------------------- | --------- | ------------------- |
| **topic** | Topic to subscribe to for diagnostics                                                    | str       | "/diagnostics" (Optional) |
| **names** | Which `DiagnosticStatus.name` values to watch; empty watches every component seen        | list[str] | [] (Optional)       |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Fault",
  "properties": {
    "seq": { "type": "integer", "minimum": 1 },
    "component": { "type": "string" },
    "event": { "type": "string", "enum": ["raise", "clear", "change"] },
    "from_level": { "type": "string", "enum": ["OK", "WARN", "ERROR", "STALE"] },
    "to_level": { "type": "string", "enum": ["OK", "WARN", "ERROR", "STALE"] },
    "previous_level_duration_s": { "type": "number", "minimum": 0 },
    "reason": { "type": "string" },
    "state": { "type": "string", "enum": ["open", "closed"] },
    "fault_started_at": { "type": "string" },
    "duration_s": { "type": "number", "minimum": 0 }
  },
  "required": ["seq", "component", "event", "from_level", "to_level", "previous_level_duration_s", "reason", "state"],
  "type": "object"
}
```

## Measurement configuration

```yaml
...
fault:
  plugin: "dc_measurements/Fault"
  topic_output: "/dc/measurement/fault"
  group_key: "fault"
  tags: ["console"]
  topic: "/diagnostics"
  names: ["motor_driver", "battery"]
```

Example Record data, a raise:

```json
{
  "seq": 1,
  "component": "motor_driver",
  "event": "raise",
  "from_level": "OK",
  "to_level": "ERROR",
  "previous_level_duration_s": 3612.4,
  "reason": "Motor fault",
  "state": "open",
  "fault_started_at": "2026-08-18T09:12:33.123456Z"
}
```

and its clear:

```json
{
  "seq": 2,
  "component": "motor_driver",
  "event": "clear",
  "from_level": "ERROR",
  "to_level": "OK",
  "previous_level_duration_s": 214.9,
  "reason": "Motor nominal",
  "state": "closed",
  "fault_started_at": "2026-08-18T09:12:33.123456Z",
  "duration_s": 214.9
}
```
