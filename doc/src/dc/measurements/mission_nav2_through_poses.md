# Mission Nav2 (NavigateThroughPoses)

## Description

The nav2 adapter of the Mission Measurement for nav2's `NavigateThroughPoses` action -- the
action a deployment issues when a mission is a single job through several hard-constraint poses
in one call, rather than a chain of separate `NavigateToPose` goals (that sibling adapter is
`mission_nav2`, #387). Emits a `mission_start` Record when a goal is first observed and a
`mission_end` Record once it reaches a terminal state, in the same Record schema #387 defines,
with `mission_type: "navigate_through_poses"`.

This Measurement is a **passive observer, not a second client competing for the action server's
single active goal**. It never sends a goal itself. Instead it subscribes to the action's own
`_action/status` and `_action/feedback` topics and calls its `_action/get_result` service
directly for whichever goal_id just reached a terminal status -- the same standard per-action
topics/services every `rclcpp_action::Server` (nav2's `bt_navigator` included) exposes. Whatever
in the deployment actually dispatches missions (a BT navigator, a fleet orchestrator, an
operator command) keeps doing so exactly as before; this Measurement only watches.

Two Records per mission:

- a **`mission_start`** Record, the first time a goal_id is observed on the action's status topic.
- a **`mission_end`** Record, once that goal_id reaches a terminal status (`SUCCEEDED`,
  `CANCELED`, or `ABORTED`) and its result has been fetched.

`outcome` on the end Record is one of `succeeded`, `failed`, `cancelled`, `aborted`:
`CANCELED` maps to `cancelled`, `ABORTED` to `aborted` (both carrying nav2's own
`error_msg`/`error_code` as `reason`/`error_code`), and `SUCCEEDED` maps to `succeeded` unless
`NavigateThroughPoses::Result.error_code` is non-zero, in which case it is `failed` -- an
application-level failure nav2 reported without aborting the goal status itself.

`recoveries` (from `NavigateThroughPoses::Feedback.number_of_recoveries`, when feedback was seen
before completion) and `waypoint_statuses` (an aggregate `{total, completed, skipped, failed}`
count of `NavigateThroughPoses::Result.waypoint_statuses` -- which poses in the job succeeded
versus were skipped or failed -- kept as counts rather than the raw per-pose array to keep the
Record flat) are both carried on the end Record when available.

A mission still running when collection stops simply never gets a matching `mission_end` Record:
nothing downstream can average an interval that was never closed as a zero, because there is no
`mission_end` Record to average. `sequence` is a monotonically increasing counter across every
Record this Measurement instance emits (start and end alike, not per mission_id), so a dropped
Record shows up as a gap rather than silently corrupting a duration or a mission-success-rate
denominator.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `action_name` | `navigate_through_poses` | The `NavigateThroughPoses` action to watch. Its `_action/status`, `_action/feedback`, and `_action/get_result` endpoints are subscribed/called directly. |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "MissionNav2ThroughPoses",
  "properties": {
    "event": { "type": "string", "enum": ["mission_start", "mission_end"] },
    "mission_id": { "type": "string" },
    "mission_type": { "type": "string" },
    "sequence": { "type": "integer", "minimum": 1 },
    "outcome": { "type": "string", "enum": ["succeeded", "failed", "cancelled", "aborted"] },
    "reason": { "type": "string" },
    "error_code": { "type": "integer", "minimum": 0 },
    "duration_sec": { "type": "number", "minimum": 0 },
    "recoveries": { "type": "integer", "minimum": 0 },
    "waypoint_statuses": {
      "type": "object",
      "properties": {
        "total": { "type": "integer", "minimum": 0 },
        "completed": { "type": "integer", "minimum": 0 },
        "skipped": { "type": "integer", "minimum": 0 },
        "failed": { "type": "integer", "minimum": 0 }
      }
    }
  },
  "required": ["event", "mission_id", "sequence"],
  "type": "object"
}
```

## Measurement configuration

```yaml
...
mission_nav2_through_poses:
  plugin: "dc_measurements/MissionNav2ThroughPoses"
  topic_output: "/dc/measurement/mission_nav2_through_poses"
  group_key: "mission"
  action_name: "navigate_through_poses"
```

Example Record data, start:

```json
{
  "event": "mission_start",
  "mission_id": "3f2504e04f8911d39a0c0305e82c3301",
  "mission_type": "navigate_through_poses",
  "sequence": 7
}
```

and end (succeeded, two of three waypoints completed):

```json
{
  "event": "mission_end",
  "mission_id": "3f2504e04f8911d39a0c0305e82c3301",
  "mission_type": "navigate_through_poses",
  "sequence": 8,
  "outcome": "succeeded",
  "duration_sec": 96.4,
  "recoveries": 1,
  "waypoint_statuses": { "total": 3, "completed": 2, "skipped": 0, "failed": 1 }
}
```

and end (aborted):

```json
{
  "event": "mission_end",
  "mission_id": "9f86d081884c7d659a2feaa0c55ad015",
  "mission_type": "navigate_through_poses",
  "sequence": 10,
  "outcome": "aborted",
  "reason": "tf timeout",
  "error_code": 9102,
  "duration_sec": 12.1
}
```
