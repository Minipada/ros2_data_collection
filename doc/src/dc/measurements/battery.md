# Battery

## Description

Records the state of one battery pack from a `sensor_msgs/BatteryState` topic: charge percentage,
voltage and current on the polling interval, plus a Record when a charging session starts and
another when it ends. Charging is unavailable time, so the session boundaries are what turn
battery data into a shift-utilisation number downstream.

The Measurement emits **facts, not metrics**: "charging started at 14:02:11 after a discharge of
62 %", never "battery availability is 87 %". Aggregation belongs in
[SQL views](../kpi_views.md), where the time window is a query parameter.

Every Record carries an `event` field naming which of the three it is:

| `event`                 | When                                             | Carries                                                                                |
| ----------------------- | ------------------------------------------------ | -------------------------------------------------------------------------------------- |
| `sample`                | Every polling interval, once the topic published | `percentage`, `voltage`, `current`, `power_supply_status`, `completed_cycles`, whatever else the pack reports |
| `charge_session_start`  | The pack starts charging                         | `session_id`, and the depth of the discharge that preceded it                          |
| `charge_session_end`    | The pack stops charging                          | `session_id`, `duration_sec`, and the percentage points gained                          |

Sessions are delimited by the pack's `power_supply_status`, never by a percentage threshold, so a
noisy percentage cannot open and close sessions repeatedly; a status of `unknown` carries no
information and leaves an open session open. Completed cycles are accumulated from discharge depth
rather than counted as full discharges — two half discharges are one cycle, not two — so a robot
topped up at every dock still reports the wear it actually did. That accounting lives in
`dc_common::BatteryCycleAccumulator`, which has no ROS dependency and is tested on its own; the
plugin subscribes, delegates and serialises.

`sensor_msgs/BatteryState` leaves most fields optional and signals "unmeasured" with `NaN`. A field
the hardware doesn't fill is **left out** of the Record rather than written as `null`, so a pack
that reports only a voltage still produces a valid Record. Until the input topic publishes at all,
no Record is emitted: a gap means no battery data, not a battery at 0 %.

Battery health is reported the way the hardware reports it — `power_supply_health` when the pack
sends one, and `health_percentage` (`capacity` against `design_capacity`) only when it sends both.

A robot with two packs runs one Measurement per pack, each with its own `topic` and
`topic_output`. Like every other Measurement, it can be gated by
[Conditions](../conditions.md) and merged into a [Group](../groups.md).

```admonish info title="Session boundaries and the polling interval"
A session boundary is queued when it happens and leaves on the next poll, one Record per poll, so
it travels the same path as every other Record (Conditions, incident buffering, Group). It keeps
the timestamp of the moment it happened, not of the poll that carried it out.
```

## Parameters

| Parameter             | Description                                                                                                     | Type   | Default                    |
| --------------------- | ---------------------------------------------------------------------------------------------------------------- | ------ | -------------------------- |
| **topic**             | Topic (`sensor_msgs/BatteryState`) to read the pack from. One Measurement per pack                               | str    | "/battery_state" (Optional) |
| **percentage_scale**  | Factor applied to the message's `percentage`. `sensor_msgs/BatteryState` specifies a 0-1 range; a driver that already publishes 0-100 is configured with `1.0` | double | 100.0 (Optional)           |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Battery",
  "description": "Battery state of one pack: charge percentage, voltage and current on the polling interval, plus a Record at each charging session boundary",
  "properties": {
    "event": { "type": "string", "enum": ["sample", "charge_session_start", "charge_session_end"] },
    "power_supply_status": { "type": "string", "enum": ["unknown", "charging", "discharging", "not_charging", "full"] },
    "percentage": { "type": "number", "minimum": 0, "maximum": 100 },
    "voltage": { "type": "number" },
    "current": { "type": "number" },
    "charge": { "type": "number" },
    "capacity": { "type": "number" },
    "design_capacity": { "type": "number" },
    "health_percentage": { "type": "number", "minimum": 0 },
    "temperature": { "type": "number" },
    "present": { "type": "boolean" },
    "power_supply_health": { "type": "string" },
    "power_supply_technology": { "type": "string" },
    "location": { "type": "string" },
    "serial_number": { "type": "string" },
    "completed_cycles": { "type": "integer", "minimum": 0 },
    "session_id": { "type": "integer", "minimum": 1 },
    "duration_sec": { "type": "number", "minimum": 0 },
    "discharge_depth_percent": { "type": "number", "minimum": 0 },
    "start_percentage": { "type": "number", "minimum": 0, "maximum": 100 },
    "end_percentage": { "type": "number", "minimum": 0, "maximum": 100 },
    "charged_percent": { "type": "number" }
  },
  "required": ["event"],
  "type": "object"
}
```

The full file (`plugins/measurements/json/battery.json`) also requires `power_supply_status` on a
`sample`; `session_id` and `discharge_depth_percent` on a session start; and `session_id` plus
`duration_sec` on a session end — the fields the views depend on. Everything else is hardware
dependent and therefore optional.

## Measurement configuration

```yaml
...
battery:
  plugin: "dc_measurements/Battery"
  topic_output: "/dc/measurement/battery"
  polling_interval: 10000
  topic: "/battery_state"
```

Two packs:

```yaml
...
battery_left:
  plugin: "dc_measurements/Battery"
  topic_output: "/dc/measurement/battery_left"
  topic: "/left/battery_state"
battery_right:
  plugin: "dc_measurements/Battery"
  topic_output: "/dc/measurement/battery_right"
  topic: "/right/battery_state"
```

Example Record data, one sample:

```json
{
  "event": "sample",
  "percentage": 62.0,
  "voltage": 48.4,
  "current": -12.5,
  "power_supply_status": "discharging",
  "power_supply_health": "good",
  "capacity": 42.0,
  "design_capacity": 50.0,
  "health_percentage": 84.0,
  "present": true,
  "completed_cycles": 3,
  "serial_number": "PACK-A"
}
```

A charging session, start and end:

```json
{
  "event": "charge_session_start",
  "session_id": 4,
  "percentage": 31.0,
  "discharge_depth_percent": 62.0
}
```

```json
{
  "event": "charge_session_end",
  "session_id": 4,
  "duration_sec": 3600.0,
  "start_percentage": 31.0,
  "end_percentage": 97.0,
  "charged_percent": 66.0
}
```
