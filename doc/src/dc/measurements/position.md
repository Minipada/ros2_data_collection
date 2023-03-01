# Position

## Description

Collect x, y and yaw of the robot.

## Parameters

| Parameter             | Description                          | Type  | Default     |
| --------------------- | ------------------------------------ | ----- | ----------- |
| **global_frame**      | Global frame                         | str   | "map"       |
| **robot_base_frame**  | Robot base frame                     | str   | "base_link" |
| **transform_timeout** | TF Timeout to use for transformation | float | 0.1         |

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Position",
    "description": "Position and orientation of the robot",
    "properties": {
        "x": {
            "description": "X position of the robot",
            "type": "number"
        },
        "y": {
            "description": "Y position of the robot",
            "type": "number"
        },
        "yaw": {
            "description": "Yaw angle of the robot",
            "type": "number"
        }
    },
    "type": "object"
}
```

## Measurement configuration

```yaml
...
position:
  plugin: "dc_measurements/Position"
  topic_output: "/dc/measurement/position"
  tags: ["flb_stdout"]
```
