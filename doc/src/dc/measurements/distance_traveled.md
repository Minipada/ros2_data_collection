# Distance traveled

## Description

Collect total distance traveled in the robot since it is powered.

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

## Measurement configuration

```yaml
...
distance_traveled:
  plugin: "dc_measurements/DistanceTraveled"
  topic_output: "/dc/measurement/distance_traveled"
  global_frame: "map"
  robot_base_frame: "base_link"
  transform_timeout: 0.1
```
