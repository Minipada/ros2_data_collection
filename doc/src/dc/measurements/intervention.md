# Intervention

## Description
Reports human takeovers: how often somebody had to step in, how long the robot had been running
itself beforehand, and how long the takeover lasted. It is a **projection of driving-mode
transitions, not a second detector** -- it reads the same mode signal
[Driving type](./driving_type.md) reads (the shared `DrivingModeSource`, so both Measurements are
configured with the same parameters) and hands it to `dc_common::StateTransitionDetector` (#360).
The two therefore cannot disagree about when the robot was autonomous.

A takeover is a transition between `autonomous` and one of the two human-driven modes, `manual` or
`teleop` -- fixed, not configurable, because `dc_kpi_intervention_events`
(`tools/infrastructure/sql/kpi_views.sql`, #369) already matches these values literally; a custom
mode name would silently never count downstream.

Two Records per takeover:

- a **start** Record, when the mode leaves `autonomous` for `manual` or `teleop`. `open` is `true`.
- an **end** Record, when the mode returns to `autonomous`. `open` is `false`.

Both carry `from_mode`, `to_mode`, and `previous_duration`: the dwell of `from_mode`, in seconds.
On a start Record that is how long the robot had been autonomous before the takeover; on an end
Record it is exactly how long the takeover itself lasted, because `from_mode` is the mode that just
ended either way. `sequence` is the transition's own monotonically increasing number (from
`StateTransitionDetector`), so a dropped Record shows up as a gap rather than silently corrupting a
duration.

A takeover still open when collection stops simply never gets a matching end Record: nothing
downstream can average an interval that was never closed as a zero, because there is no `end`
Record to average. Nothing is published on a poll that saw no takeover boundary, so unlike
`driving_type` this Measurement is silent most of the time. A transition that is not a boundary --
autonomy handing over to `unknown` because the mode signal went stale, or one human mode replacing
another directly -- produces no Record.

Rates (interventions per autonomous hour, per kilometre travelled) are deliberately **not**
computed here: they are views over these Records (`dc_kpi_intervention_rate`), so the denominator
can change without touching a robot. Set `group_key` to merge an intervention with `position` in a
[Group](../groups.md) and put the takeover on the site map.

## Parameters

The mode-source parameters are the same ones [Driving type](./driving_type.md) takes, with the
same meaning: `mode_topic`, `value_mapping_from`, `value_mapping_to`, `velocity_topics`,
`velocity_modes`, `velocity_timeout_s`. Configuring both `mode_topic` and `velocity_topics` is a
configuration error.

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Intervention",
  "description": "A human takeover, derived from driving-mode transitions: one Record when it starts, one when it ends",
  "properties": {
    "event": { "type": "string", "enum": ["start", "end"] },
    "from_mode": { "type": "string", "enum": ["autonomous", "manual", "teleop", "unknown"] },
    "to_mode": { "type": "string", "enum": ["autonomous", "manual", "teleop", "unknown"] },
    "previous_duration": { "type": "number", "minimum": 0 },
    "sequence": { "type": "integer", "minimum": 1 },
    "open": { "type": "boolean" }
  },
  "required": ["event", "from_mode", "to_mode", "previous_duration", "sequence", "open"],
  "type": "object"
}
```

## Measurement configuration

```yaml
...
intervention:
  plugin: "dc_measurements/Intervention"
  topic_output: "/dc/measurement/intervention"
  group_key: "intervention"
  tags: ["console"]
  velocity_topics: ["/cmd_vel_smoothed", "/teleop/cmd_vel"]
  velocity_modes: ["autonomous", "teleop"]
  velocity_timeout_s: 1.0
```

Example Record data, start:

```json
{
  "event": "start",
  "from_mode": "autonomous",
  "to_mode": "teleop",
  "previous_duration": 412.5,
  "sequence": 7,
  "open": true
}
```

and end:

```json
{
  "event": "end",
  "from_mode": "teleop",
  "to_mode": "autonomous",
  "previous_duration": 37.2,
  "sequence": 8,
  "open": false
}
```
