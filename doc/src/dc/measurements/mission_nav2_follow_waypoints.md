# Mission (nav2 FollowWaypoints)

## Description

Reports the outcome of a nav2 `FollowWaypoints` mission -- a patrol or waypoint-following run,
including nav2's own per-waypoint failure reporting -- as one `mission_start` Record when a goal is
accepted and one `mission_end` Record when it reaches a terminal state. It is the `FollowWaypoints`
sibling of the `NavigateToPose` Mission Measurement (#387): same Record schema, same
`mission_id`/`sequence` conventions, its own `mission_type`.

This Measurement is a **passive watcher, not a commander**: it never sends a `FollowWaypoints` goal
itself. Whatever already dispatches waypoint-following missions on the robot --
[`nav2_simple_commander`](https://docs.nav2.org/commander_api/index.html), a WMS integration, a
teleop panel -- keeps doing exactly that; this Measurement only observes. That is a deliberate
match to every other Measurement's read-only relationship to the systems it reports on, and it is
also why it does not use `rclcpp_action::Client`'s typed goal-tracking API: that API only reports
on goals the client itself sent. Instead it subscribes directly to the action's `_action/status`
topic (`action_msgs/msg/GoalStatusArray`, the same message type for every action) to see a goal get
accepted or reach a terminal state, and calls the action's `_action/get_result` service directly
once it does, to fetch the Result -- both standard, public parts of the ROS 2 action wire protocol
that any client may use, sender or not.

## Known limitation: `number_of_loops`

nav2's `FollowWaypoints` goal carries a `number_of_loops` field (how many times to repeat the
route), but the goal itself is never re-published anywhere a third party can observe it -- only the
sender and the action server ever see it, and nothing in the ROS 2 action protocol (nor
`rclcpp_action::Client`'s public API) lets a client that did not send a goal read it back. A
passive watcher therefore cannot report `number_of_loops`, and this Measurement's Records do not
carry it. Sending the goal itself instead of watching it would make this Measurement responsible
for driving navigation, contradicting DC's role as a telemetry pipeline (see `CONTEXT.md`) and
directly competing with whatever else already commands the same action -- e.g.
`dc_demos/dc_demos/qrcodes_waypoint_follower.py`, which already sends `FollowWaypoints` goals of
its own. If a deployment needs `number_of_loops` on this Record, it has to come from the mission
commander via a documented escape hatch (#305's territory), not from this adapter.

## Parameters

| Parameter     | Type   | Default             | Description                                                        |
| ------------- | ------ | -------------------- | -------------------------------------------------------------------- |
| `action_name` | string | `follow_waypoints`   | The nav2 `FollowWaypoints` action to watch                          |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "MissionNav2FollowWaypoints",
  "properties": {
    "event": { "type": "string", "enum": ["mission_start", "mission_end"] },
    "mission_id": { "type": "string" },
    "mission_type": { "type": "string", "const": "follow_waypoints" },
    "sequence": { "type": "integer", "minimum": 1 },
    "outcome": { "type": "string", "enum": ["succeeded", "failed", "cancelled", "aborted"] },
    "reason": { "type": "string" },
    "error_code": { "type": "integer", "minimum": 0 },
    "duration_sec": { "type": "number", "minimum": 0 },
    "missed_waypoints": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "index": { "type": "integer", "minimum": 0 },
          "error_code": { "type": "integer", "minimum": 0 }
        },
        "required": ["index", "error_code"]
      }
    }
  },
  "required": ["event", "mission_id", "mission_type", "sequence"],
  "type": "object"
}
```

`outcome`, `duration_sec` and `missed_waypoints` are required on `mission_end` only. `reason` and
`error_code` are required on `mission_end` when `outcome` is `failed` or `aborted`: `failed` is a
goal that reached nav2's `GoalStatus.SUCCEEDED` but whose `FollowWaypoints::Result.error_code` is
still non-zero -- nav2's `WaypointFollower` can finish a route having missed a waypoint without the
action itself aborting.

## Measurement configuration

```yaml
...
mission:
  plugin: "dc_measurements/MissionNav2FollowWaypoints"
  topic_output: "/dc/measurement/mission"
  group_key: "mission"
  action_name: "follow_waypoints"
```

Example Record data, start:

```json
{
  "event": "mission_start",
  "mission_id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "mission_type": "follow_waypoints",
  "sequence": 5
}
```

end, succeeded:

```json
{
  "event": "mission_end",
  "mission_id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "mission_type": "follow_waypoints",
  "sequence": 6,
  "outcome": "succeeded",
  "duration_sec": 184.2,
  "missed_waypoints": []
}
```

and end, failed on a missed waypoint:

```json
{
  "event": "mission_end",
  "mission_id": "6c9f6a3e-2d1a-4e3a-9f7a-1b2c3d4e5f60",
  "mission_type": "follow_waypoints",
  "sequence": 8,
  "outcome": "failed",
  "reason": "task executor failed",
  "error_code": 601,
  "duration_sec": 92.6,
  "missed_waypoints": [
    { "index": 2, "error_code": 601 }
  ]
}
```

`error_code` is `FollowWaypoints::Result.error_code` verbatim -- on the Jazzy `nav2_msgs` this repo targets, its only non-zero values are `UNKNOWN` (600) and `TASK_EXECUTOR_FAILED` (601); `missed_waypoints[].error_code` is nav2's `MissedWaypoint.error_code` for that one waypoint, from the same table. `MissedWaypoint` on Jazzy carries only `index`, the waypoint's goal pose (not part of this schema) and `error_code` -- no per-waypoint status enum or free-text reason the way a newer/rolling nav2 has; `reason`/`error_msg` above are always the mission-level `FollowWaypoints::Result.error_msg`, not anything per-waypoint.
