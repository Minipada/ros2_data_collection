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
