# Double equal

## Description

Compare JSON key value to the value passed in parameter and returns true if equal.

## Parameters

| Parameter | Description                                                            | Type  | Default         |
| --------- | ---------------------------------------------------------------------- | ----- | --------------- |
| **key**   | JSON key where value is located, separate nested dictionary with **/** | str   | N/A (Mandatory) |
| **value** | Value to which compare the JSON value                                  | float | N/A (Mandatory) |

## Configuration

A Condition is evaluated against the Record of the Measurement it gates, so `key` names
one of that same Measurement's own fields. Here, only publish
[Position](../measurements/position.md) while the robot is facing its calibrated
reference heading exactly (`yaw: 0.0`) — useful right after a docking or calibration
routine:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["facing_reference_heading"]
    facing_reference_heading:
      plugin: "dc_conditions/DoubleEqual"
      key: "yaw"
      value: 0.0
    position:
      plugin: "dc_measurements/Position"
      if_all_conditions: ["facing_reference_heading"]
      topic_output: "/dc/measurement/position"
```
