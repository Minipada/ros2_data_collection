# Mission Nav2 (NavigateToPose)

## Description

The nav2 adapter of the Mission Measurement for nav2's `NavigateToPose` action -- a single-pose
navigation goal, the base case the mission lifecycle contract (#305, recorded in
[ADR-0010](../../adr/0010-mission-lifecycle-contract.md)) was agreed against. Emits a
`mission_start` Record when a goal is accepted and a `mission_end` Record once it reaches a
terminal state. `NavigateThroughPoses` and `FollowWaypoints` are its siblings (#388/#389): same
Record schema, same `mission_id`/`sequence` conventions, each with its own `mission_type`.

This Measurement is a **passive watcher, not a commander**: it never sends a `NavigateToPose` goal
itself. Whatever already dispatches navigation missions on the robot -- nav2's `bt_navigator`, a
fleet orchestrator, an operator command -- keeps doing exactly that; this Measurement only
observes. That is a deliberate match to every other Measurement's read-only relationship to the
systems it reports on, and it is also why it does not use `rclcpp_action::Client`'s typed
goal-tracking API: that API only reports on goals the client itself sent. Instead it subscribes
directly to the action's `_action/status` (`action_msgs/msg/GoalStatusArray`, the same message
type for every action) and `_action/feedback` topics to see a goal get accepted, pick up its
recovery count, and reach a terminal state, and calls the action's `_action/get_result` service
directly once it does, to fetch the Result -- all standard, public parts of the ROS 2 action wire
protocol that any client may use, sender or not.

nav2's `NavigateToPose` action server processes one goal at a time, so this Measurement only ever
tracks one mission at once. A second goal accepted before the first reaches a terminal state is
logged and otherwise ignored -- it produces no Record of its own, and the mission already being
tracked is unaffected.

`outcome` on the end Record is one of `succeeded`, `failed`, `cancelled`, `aborted`: `CANCELED`
maps to `cancelled`, `ABORTED` to `aborted` (both carrying nav2's own `error_msg`/`error_code` as
`reason`/`error_code`), and `SUCCEEDED` maps to `succeeded` unless
`NavigateToPose::Result.error_code` is non-zero, in which case it is `failed` -- an
application-level failure nav2 reported without aborting the goal status itself.

`recoveries` (from `NavigateToPose::Feedback.number_of_recoveries`, when feedback was seen before
completion) is carried on the end Record when available.

A mission still running when collection stops simply never gets a matching `mission_end` Record:
nothing downstream can average an interval that was never closed as a zero, because there is no
`mission_end` Record to average. `sequence` is a monotonically increasing counter across every
Record this Measurement instance emits (start and end alike, not per mission_id), so a dropped
Record shows up as a gap rather than silently corrupting a duration or a mission-success-rate
denominator.

## Parameters

| Parameter     | Type   | Default            | Description                        |
| ------------- | ------ | ------------------- | ------------------------------------ |
| `action_name` | string | `navigate_to_pose`  | The nav2 `NavigateToPose` action to watch |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "MissionNav2",
  "properties": {
    "event": { "type": "string", "enum": ["mission_start", "mission_end"] },
    "mission_id": { "type": "string" },
    "mission_type": { "type": "string", "const": "navigate_to_pose" },
    "sequence": { "type": "integer", "minimum": 1 },
    "outcome": { "type": "string", "enum": ["succeeded", "failed", "cancelled", "aborted"] },
    "reason": { "type": "string" },
    "error_code": { "type": "integer", "minimum": 0 },
    "duration_sec": { "type": "number", "minimum": 0 },
    "recoveries": { "type": "integer", "minimum": 0 }
  },
  "required": ["event", "mission_id", "mission_type", "sequence"],
  "type": "object"
}
```

`outcome` and `duration_sec` are required on `mission_end` only. `reason` and `error_code` are
required on `mission_end` when `outcome` is `failed` or `aborted`.

## Measurement configuration

```yaml
...
mission:
  plugin: "dc_measurements/MissionNav2"
  topic_output: "/dc/measurement/mission"
  group_key: "mission"
  action_name: "navigate_to_pose"
```

Example Record data, start:

```json
{
  "event": "mission_start",
  "mission_id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "mission_type": "navigate_to_pose",
  "sequence": 7
}
```

and end (succeeded):

```json
{
  "event": "mission_end",
  "mission_id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "mission_type": "navigate_to_pose",
  "sequence": 8,
  "outcome": "succeeded",
  "duration_sec": 96.4,
  "recoveries": 1
}
```

and end (aborted):

```json
{
  "event": "mission_end",
  "mission_id": "6c9f6a3e-2d1a-4e3a-9f7a-1b2c3d4e5f60",
  "mission_type": "navigate_to_pose",
  "sequence": 10,
  "outcome": "aborted",
  "reason": "tf timeout",
  "error_code": 9102,
  "duration_sec": 12.1
}
```
