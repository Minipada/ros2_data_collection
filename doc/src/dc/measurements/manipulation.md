# Manipulation

## Description

Reports one MoveIt [`MoveGroup`](https://github.com/moveit/moveit_msgs/blob/master/action/MoveGroup.action)
goal's lifecycle: a `manipulation_start` Record when the goal is accepted, a `manipulation_end`
Record when it reaches a terminal state. This is a **manipulation goal, not a Mission** -- it
moves a robot arm for one planning group, not a fleet task, and MoveIt's outcome vocabulary is its
own: one flat signed `MoveItErrorCodes` space, not nav2's succeeded/failed/cancelled/aborted
split.

The Measurement is a **passive observer**, not the client that sends the goal. It reads the
action's status topic (`<action_name>/_action/status`, published for every goal the server knows
about, regardless of which client sent it) to see a goal appear and reach a terminal state, then
calls the action's `get_result` service -- a plain service any client may call given the goal's
UUID, not only the one that sent it -- to read `MoveGroup::Result`'s `error_code` and
`planning_time`. `group_name` is this instance's own configuration rather than something read off
the goal: `MoveGroup`'s `Goal` (the only place a group name appears) is sent privately to the
action server and never broadcast, so a passive observer structurally cannot see it. Run one
Manipulation Measurement instance per planning group your robot moves.

`outcome` on the end Record is derived from `error_code`, not from the action's own terminal
status: `SUCCESS` maps to `succeeded`, `PREEMPTED` maps to `cancelled` (the closest thing MoveIt
has), and every other -- necessarily negative -- `MoveItErrorCodes` value maps to `failed`.
`sequence` is a monotonically increasing counter across every Record this instance emits, so a
dropped Record shows up as a gap rather than silently corrupting a downstream count.

A goal still executing when collection stops simply never gets a matching `manipulation_end`
Record: nothing downstream can average an interval that was never closed as a zero, because there
is no `end` Record to average.

## Parameters

| Parameter     | Description                                                    | Default        |
| ------------- | ---------------------------------------------------------------| -------------- |
| `action_name` | The `MoveGroup` action to watch                                | `move_action`  |
| `group_name`  | The planning group this instance reports on the emitted Record | none, required |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Manipulation",
  "description": "One MoveIt MoveGroup goal's lifecycle: a Record when it is accepted, one when it reaches a terminal state",
  "properties": {
    "event": {
      "description": "Whether this Record is the start or the end of one manipulation goal",
      "type": "string",
      "enum": ["manipulation_start", "manipulation_end"]
    },
    "goal_id": {
      "description": "The MoveGroup goal's own UUID, as a canonical hex string, correlating a start Record with its end",
      "type": "string"
    },
    "group_name": {
      "description": "The planning group this Measurement instance watches, as configured -- not read off the goal, which is never broadcast",
      "type": "string"
    },
    "sequence": {
      "description": "Monotonically increasing across every Record from this Measurement instance, so a dropped Record shows up as a gap",
      "type": "integer",
      "minimum": 1
    },
    "outcome": {
      "description": "Derived from MoveIt's own error_code: SUCCESS succeeds, PREEMPTED is the closest thing MoveIt has to cancelled, everything else fails",
      "type": "string",
      "enum": ["succeeded", "cancelled", "failed"]
    },
    "error_code": {
      "description": "MoveIt's own numeric MoveItErrorCodes, carried through verbatim",
      "type": "integer"
    },
    "planning_time": {
      "description": "Seconds MoveGroup spent planning, from MoveGroup::Result",
      "type": "number",
      "minimum": 0
    },
    "duration_sec": {
      "description": "How long the goal took from accepted to terminal",
      "type": "number",
      "minimum": 0
    }
  },
  "required": ["event", "goal_id", "group_name", "sequence"],
  "type": "object"
}
```

## Measurement configuration

```yaml
...
manipulation:
  plugin: "dc_measurements/Manipulation"
  topic_output: "/dc/measurement/manipulation"
  group_key: "manipulation"
  tags: ["console"]
  action_name: "move_action"
  group_name: "arm"
```

Example Record data, start:

```json
{
  "event": "manipulation_start",
  "goal_id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "group_name": "arm",
  "sequence": 5
}
```

and end:

```json
{
  "event": "manipulation_end",
  "goal_id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "group_name": "arm",
  "sequence": 6,
  "outcome": "succeeded",
  "error_code": 1,
  "planning_time": 0.842,
  "duration_sec": 3.15
}
```
