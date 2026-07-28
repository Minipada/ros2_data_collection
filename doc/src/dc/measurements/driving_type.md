# Driving type

## Description
Reports the robot's current operating mode (`autonomous`, `manual`, `teleop`, or `unknown`) as a
single Record, so every other Measurement can be segmented by driving mode downstream. There is no
standard ROS message for "current driving mode", so this Measurement is entirely configuration
driven and supports two common shapes, chosen by which parameters are set (configuring both is a
configuration error):

- **Dedicated mode topic**: subscribes to `mode_topic` (`std_msgs/String`) and maps each raw value
  it carries to a mode through `value_mapping_from`/`value_mapping_to`. A raw value with no entry
  in the mapping is ignored (the previous mode is kept) rather than treated as `unknown`, since an
  unrecognized value is more likely an upstream hiccup than an actual mode change.
- **Velocity source inference**: subscribes to `velocity_topics` (`geometry_msgs/Twist`, e.g. one
  topic per command source such as a Nav2 output and a joystick teleop node) and reports the mode
  of whichever configured source last published, mapped through the parallel `velocity_modes`
  list. A source that hasn't published within `velocity_timeout_s` is no longer considered active.

The emitted `mode` is always one of the four values above -- a documented, closed set -- so
downstream grouping/dashboards never see an unbounded string. Before any mode has been observed
(no Measurement configured, a dedicated mode topic that hasn't published yet, or every velocity
source past its timeout) the Measurement reports `"unknown"` rather than skipping the Record: a
Record is always published on every poll, so a gap in `driving_type` data means the plugin itself
stopped, not "mode currently unknown".

## Parameters

| Parameter               | Description                                                                                                   | Type        | Default   |
| ------------------------ | -------------------------------------------------------------------------------------------------------------- | ----------- | --------- |
| **mode_topic**            | Topic (`std_msgs/String`) carrying a raw mode value, mapped through `value_mapping_from`/`value_mapping_to`. Mutually exclusive with `velocity_topics` | str         | "" (Optional) |
| **value_mapping_from**    | Raw values received on `mode_topic`, aligned by index with `value_mapping_to`                                   | list of str | [] (Optional) |
| **value_mapping_to**      | Mode each `value_mapping_from` entry maps to; must be one of `autonomous`, `manual`, `teleop`, `unknown`         | list of str | [] (Optional) |
| **velocity_topics**       | Velocity command topics (`geometry_msgs/Twist`) to infer the mode from, aligned by index with `velocity_modes`. Mutually exclusive with `mode_topic` | list of str | [] (Optional) |
| **velocity_modes**        | Mode each `velocity_topics` entry reports while it's the most recently active source                            | list of str | [] (Optional) |
| **velocity_timeout_s**    | Seconds since a velocity source's last message before it's no longer considered active                         | double      | 1.0 (Optional) |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "DrivingType",
  "description": "Current driving/operating mode of the robot, so every other metric can be segmented by mode",
  "properties": {
    "mode": {
      "description": "Driving mode: 'unknown' until a mode has been observed (dedicated mode topic) or a configured velocity source has published within 'velocity_timeout_s' (velocity-source inference)",
      "type": "string",
      "enum": ["autonomous", "manual", "teleop", "unknown"]
    }
  },
  "required": ["mode"],
  "type": "object"
}
```

## Measurement configuration

Dedicated mode topic:

```yaml
...
driving_type:
  plugin: "dc_measurements/DrivingType"
  topic_output: "/dc/measurement/driving_type"
  tags: ["console"]
  mode_topic: "/driving_mode_raw"
  value_mapping_from: ["0", "1", "2"]
  value_mapping_to: ["manual", "autonomous", "teleop"]
```

Velocity source inference:

```yaml
...
driving_type:
  plugin: "dc_measurements/DrivingType"
  topic_output: "/dc/measurement/driving_type"
  tags: ["console"]
  velocity_topics: ["/nav2/cmd_vel", "/teleop/cmd_vel"]
  velocity_modes: ["autonomous", "teleop"]
  velocity_timeout_s: 1.0
```

Example Record data:

```json
{
  "mode": "autonomous"
}
```
