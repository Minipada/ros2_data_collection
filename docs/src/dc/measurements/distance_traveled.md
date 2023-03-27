# Distance traveled

## Description

Collect total distance traveled in the robot since it is powered.

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
  tags: ["flb_stdout"]
  global_frame: "map"
  robot_base_frame: "base_link"
  transform_timeout: 0.1
```
