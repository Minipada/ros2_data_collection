# Cmd_vel

## Description

Collect command velocity sent to the robot by subscribing to cmd_vel topic.

## Parameters

| Parameter | Description                                                    | Type | Default    |
| --------- | -------------------------------------------------------------- | ---- | ---------- |
| **topic** | Topic to subscribe to to get the cmd_vel (geometry_msgs/Twist) | str  | "/cmd_vel" |

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Cmd_vel",
    "description": "Command velocity sent to the robot",
    "properties": {
        "computed": {
            "description": "Computed command velocity in meter/s",
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
cmd_vel:
  plugin: "dc_measurements/CmdVel"
  group_key: "cmd_vel"
  topic_output: "/dc/measurement/cmd_vel"
```
