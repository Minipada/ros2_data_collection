# Speed

## Description

Collect robot speed using the Odom topic.

## Parameters

| Parameter      | Description                                                   | Type | Default |
| -------------- | ------------------------------------------------------------- | ---- | ------- |
| **odom_topic** | Topic to subscribe to to get the odometry (nav_msgs/Odometry) | str  | "/odom" |

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Speed",
    "description": "Computed, linear and angular speed of the robot",
    "properties": {
        "computed": {
            "description": "Computed speed in meter/s",
            "type": "number"
        },
        "linear": {
            "description": "Linear velocity as a vector",
            "type": "object",
            "items": {
                "$ref": "#/$defs/vector3"
            }
        },
        "angular": {
            "description": "Angular velocity as a vector",
            "type": "object",
            "items": {
                "$ref": "#/$defs/vector3"
            }
        }
    },
    "$defs": {
        "vector3": {
            "type": "object",
            "properties": {
                "x": {
                    "description": "X speed",
                    "type": "number"
                },
                "y": {
                    "description": "Y speed",
                    "type": "number"
                },
                "z": {
                    "description": "Z speed",
                    "type": "number"
                }
            }
        }
    },
    "type": "object"
}
```

## Measurement configuration

```yaml
...
speed:
  plugin: "dc_measurements/Speed"
  topic_output: "/dc/measurement/speed"
  tags: ["flb_stdout"]
  odom_topic: "/odom"
```
