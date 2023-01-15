# Map

## Description

Save map using nav2_map_server and collect the map of the local map saved. The measurement also includes metadata: width, heigh and x and y origin.

## Parameters

| Parameter     | Description                                                                     | Type | Default                 |
| ------------- | ------------------------------------------------------------------------------- | ---- | ----------------------- |
| **topic**     | Topic to subscribe to to get the map                                            | str  | "/map"                  |
| **save_path** | Path to save the map to. Environment variables and datetime format are expanded | str  | "map/%Y-%m-%dT%H:%M:%S" |

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Map",
    "description": "Map saved metadata and paths",
    "properties": {
        "resolution": {
            "description": "Resolution of the map, meters/pixel",
            "type": "number",
            "minimum": 0
        },
        "local_paths": {
            "description": "Paths where metadata and image are stored",
            "type": "object",
            "items": {
                "$ref": "#/$defs/paths"
            }
        },
        "origin": {
            "description": "Robot origin position in meters",
            "type": "object",
            "items": {
                "$ref": "#/$defs/origin"
            }
        },
        "width": {
            "description": "Width of the PGM",
            "type": "integer",
            "minimum": 0
        },
        "height": {
            "description": "Height of the PGM",
            "type": "integer",
            "minimum": 0
        }
    },
    "$defs": {
        "origin": {
            "type": "object",
            "description": "The 2-D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise rotation (yaw=0 means no rotation). Many parts of the system currently ignore yaw.",
            "properties": {
                "x": {
                    "description": "X origin of the robot",
                    "type": "number"
                },
                "y": {
                    "description": "Y origin of the robot",
                    "type": "number"
                }
            }
        },
        "paths": {
            "type": "object",
            "properties": {
                "yaml": {
                    "description": "Path to the map YAML file containing map metadata",
                    "type": "string"
                },
                "pgm": {
                    "description": "Path to the map PGM file containing the gray-scale image",
                    "type": "string"
                }
            }
        }
    },
    "type": "object"
}
```
