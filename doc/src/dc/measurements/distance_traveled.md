# Distance traveled

## Description

Reports the straight-line distance moved between the robot's current TF pose
(`robot_base_frame` in `global_frame`) and its pose at the *previous* poll — not a running
odometer total, despite the field's name. Sum it downstream (e.g. `SUM(distance_traveled)`
in a SQL view) if a cumulative total since power-on is what you actually want.

```admonish warning title="distance_traveled is a per-poll delta, not a running total"
`collect()` computes `sqrt((x - last_x)^2 + (y - last_y)^2)` against the position recorded
on the *previous* poll and overwrites `last_x_`/`last_y_` with the current one — it never
accumulates into a sum. A stationary robot reports `0.0` every poll; a robot that moved 3 m
since the last poll reports `3.0` once, then `0.0` again once it stops. The Measurement's own
name and its schema's `"description": "Total distance traveled in meters"` both suggest a
running total; neither matches what the code does. Known, not yet fixed or renamed.
```

## Parameters

| Parameter             | Description                          | Type  | Default     |
| --------------------- | ------------------------------------ | ----- | ----------- |
| **global_frame**      | Global frame                         | str   | "map"       |
| **robot_base_frame**  | Robot base frame                     | str   | "base_link" |
| **transform_timeout** | TF Timeout to use for transformation | float | 0.1         |

```admonish warning title="transform_timeout is not actually configurable"
`onConfigure()` declares a `transform_tolerance` parameter (default `0.1`) but then reads back
`transform_timeout` — a name that was never declared — into the member this Measurement actually
uses. `rclcpp`'s `get_parameter(name, out)` silently no-ops on an undeclared name rather than
throwing, so neither key you might set in YAML reaches the TF lookup: `transform_tolerance` is
declared but never read, and `transform_timeout` is read but never declared. The member is left
uninitialized (`float transform_timeout_;`, no default), so the TF-lookup timeout actually used at
runtime is whatever that memory happened to contain, not the `0.1` shown above. Known, tracked in
`dc_measurements/plugins/measurements/distance_traveled.cpp`, not yet fixed.
```

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Distance traveled",
    "description": "Total distance traveled in meters by the robot",
    "properties": {
        "distance_traveled": {
            "description": "Total distance traveled in meters",
            "type": "number"
        }
    },
    "type": "object"
}
```

## Configuration

```yaml
...
distance_traveled:
  plugin: "dc_measurements/DistanceTraveled"
  topic_output: "/dc/measurement/distance_traveled"
  global_frame: "map"
  robot_base_frame: "base_link"
  transform_timeout: 0.1
```

## Example output

The poll during which the robot's TF pose moved 3 m since the previous poll:

```json
{
  "distance_traveled": 3.0,
  "flattened": false,
  "name": "distance_traveled",
  "nested": false,
  "run_id": "169"
}
```

The next poll, stationary:

```json
{
  "distance_traveled": 0.0,
  "flattened": false,
  "name": "distance_traveled",
  "nested": false,
  "run_id": "169"
}
```
