# Mission (Open-RMF)

## Description

The Open-RMF adapter of the Mission Measurement for Open-RMF's `TaskState` (the document
`rmf-web`'s API server maintains per task). Emits a `mission_start` Record when a task's status
first leaves `queued`/`standby` into an active state, and a `mission_end` Record once it reaches a
terminal status, in the same `mission_start`/`mission_end` Record schema #387 defines --
`mission_id` sourced from Open-RMF's own `booking.id`, `mission_type` from `category` (Open-RMF
tasks have a real category/type, unlike nav2's bare goal).

Unlike the nav2 adapters (`mission_nav2`/`mission_nav2_follow_waypoints`/
`mission_nav2_through_poses`), Open-RMF exposes no ROS action or topic for task lifecycle. This
Measurement instead consumes `TaskState` as a stream of JSON documents over a plain `ws://`
connection, using the reusable `dc_common::WebSocketJsonClient`. **The websocket protocol
`rmf-web`'s API server actually speaks on its own endpoints (Socket.IO framing) is out of scope**
-- `websocket_url` must point at an endpoint that re-emits the task-state feed as one `TaskState`
JSON object per plain text frame; bridging the real `rmf-web` wire protocol is a separate piece of
infrastructure this Measurement assumes already exists, not something it does itself.

This Measurement is a **passive observer**. It never dispatches, cancels, or otherwise mutates any
Open-RMF task -- it only watches the state stream.

## Status mapping

Open-RMF's task model is not a flat start/end pair: `TaskState.status` is a continuously-updated
12-value enum (`uninitialized`, `blocked`, `error`, `failed`, `queued`, `standby`, `underway`,
`delayed`, `skipped`, `canceled`, `killed`, `completed`), not a single terminal signal. Two Records
per mission are still produced:

- a **`mission_start`** Record, the first time a `booking.id` is observed leaving
  `queued`/`standby` into an active state -- `underway`, `delayed`, or (once already active)
  `blocked`/`error`.
- a **`mission_end`** Record, once that `booking.id` reaches a terminal status: `completed`,
  `failed`, `canceled`, `killed`, or `skipped`.

`outcome` on the end Record is one of `succeeded`, `failed`, `cancelled`, `aborted`:
`completed`→`succeeded`, `failed`→`failed`, `canceled`→`cancelled`, `killed`→`aborted`.

Two cases the acceptance criteria asked to be resolved explicitly, not silently defaulted:

- **`skipped` is treated as task-terminal**, and maps to outcome **`cancelled`**. It is defined in
  the same `status` enum `task_state.json` uses for the task's own top-level `status` field (not a
  separate phase-only enum), so a task can legitimately end its life with `status: "skipped"`.
  Open-RMF's `skipped` means the task's work was bypassed rather than performed -- closer to DC's
  `cancelled` (closed without completing the work, not an error) than to `succeeded` (implies the
  work was done) or a failure outcome. DC's outcome contract (#305/#387) has four values, not
  five; this Measurement does not add a fifth `skipped` outcome to it.
- **`blocked` and `error` are treated as transient, not terminal.** Both describe a task Open-RMF
  is still actively trying to resolve or recover (a blocked path, a recoverable fault), not a
  status its task manager ever settles on as the end of the task's life -- they are absent from
  `task_state.json`'s terminal set. A task observed as `blocked`/`error` stays open; only a later
  terminal status closes it, and it may still resolve back to `underway`.

`reason` is populated on a best-effort basis, carried verbatim from whichever part of `TaskState`
Open-RMF actually populated for that outcome -- `dispatch.errors` (or the top-level `detail`) for
`failed`, `cancellation.labels` for `cancelled`, `killed.labels` for `aborted`. Unlike nav2's
always-present `error_msg`, Open-RMF does not guarantee one of these for every outcome, so `reason`
(and `error_code`, only ever populated for `failed`) may be absent even on a `mission_end` Record.

`duration_sec` prefers Open-RMF's own `unix_millis_start_time`/`unix_millis_finish_time` when the
source provided both; it falls back to this Measurement's own locally observed start/end
timestamps otherwise.

## What this Measurement does not represent

- **No phase-level detail.** `TaskState.phases`/`active`/`completed`/`pending` (which step of a
  multi-phase task is running) is not carried into the Record. Only the task's own top-level
  `status` drives `mission_start`/`mission_end`.
- **No interruptions.** `TaskState.interruptions` (temporary holds placed on a task, distinct from
  cancellation) is not represented at all -- an interrupted-then-resumed task simply keeps running
  from this Measurement's point of view.
- **No dispatch/assignment detail.** Which fleet or robot a task was assigned to
  (`TaskState.assigned_to`/`dispatch.assignment`) is not carried into the Record.

A task still active when the websocket connection drops or collection stops simply never gets a
matching `mission_end` Record: nothing downstream can average an interval that was never closed,
because there is no `mission_end` Record to average -- matching #387's open-interval handling.
Symmetrically, a task whose first-ever observed sample is already active (started before this
Measurement connected) or already terminal (finished before this Measurement connected) is not
tracked at all: no `mission_start` Record (the active-transition boundary was never observed) and
consequently no `mission_end` Record either.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `websocket_url` | *(required)* | `ws://host[:port][/path]` of the endpoint streaming `TaskState` JSON, one object per text frame. |
| `reconnect_initial_backoff_ms` | `500` | Delay before the first reconnect attempt after a dropped connection. |
| `reconnect_max_backoff_ms` | `30000` | Reconnect delay never grows past this, however many attempts fail in a row. |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "MissionOpenRmf",
  "properties": {
    "event": { "type": "string", "enum": ["mission_start", "mission_end"] },
    "mission_id": { "type": "string" },
    "mission_type": { "type": "string" },
    "sequence": { "type": "integer", "minimum": 1 },
    "outcome": { "type": "string", "enum": ["succeeded", "failed", "cancelled", "aborted"] },
    "reason": { "type": "string" },
    "error_code": { "type": "integer", "minimum": 0 },
    "duration_sec": { "type": "number", "minimum": 0 }
  },
  "required": ["event", "mission_id", "sequence"],
  "type": "object"
}
```

## Measurement configuration

```yaml
...
mission_open_rmf:
  plugin: "dc_measurements/MissionOpenRmf"
  topic_output: "/dc/measurement/mission_open_rmf"
  group_key: "mission"
  websocket_url: "ws://rmf-task-state-bridge:8080/task_states"
```

Example Record data, start:

```json
{
  "event": "mission_start",
  "mission_id": "delivery.dispenser_1.dispatch-14",
  "mission_type": "delivery",
  "sequence": 3
}
```

end (succeeded):

```json
{
  "event": "mission_end",
  "mission_id": "delivery.dispenser_1.dispatch-14",
  "mission_type": "delivery",
  "sequence": 4,
  "outcome": "succeeded",
  "duration_sec": 214.7
}
```

end (cancelled, cancellation labels present):

```json
{
  "event": "mission_end",
  "mission_id": "patrol.loop_a.dispatch-22",
  "mission_type": "patrol",
  "sequence": 9,
  "outcome": "cancelled",
  "reason": "operator; dashboard",
  "duration_sec": 42.1
}
```

end (failed, dispatch error present):

```json
{
  "event": "mission_end",
  "mission_id": "delivery.dispenser_2.dispatch-31",
  "mission_type": "delivery",
  "sequence": 15,
  "outcome": "failed",
  "reason": "dispenser_unavailable: dispenser_1 did not respond",
  "error_code": 12,
  "duration_sec": 8.9
}
```
