# Map

## Description

Save map using nav2_map_server and collect the map of the local map saved. The measurement also includes metadata: width, height and x and y origin.

## Parameters

| Parameter            | Description                                                                     | Type  | Default                 |
| -------------------- | ------------------------------------------------------------------------------- | ----- | ----------------------- |
| **quiet**            | Disable stdout for nav2 map saver                                               | bool  | true                    |
| **save_base64**      | Also save the PNG image as a base64 string, under `base64.png`                  | bool  | false                   |
| **save_path**        | Path to save the map to. Environment variables and datetime format are expanded | str   | "map/%Y-%m-%dT%H:%M:%S" |
| **save_map_timeout** | Time to wait to save the map                                                    | float | 3.0                     |
| **topic**            | Topic to subscribe to to get the map                                            | str   | "/map"                  |

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
        "remote_paths": {
            "description": "Dictionary of paths where metadata and image will be remotely stored",
            "type": "object",
            "additionalProperties": {
                "type": "object",
                "items": {
                    "$ref": "#/$defs/paths"
                }
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
                "png": {
                    "description": "Path to the map PNG file containing the image",
                    "type": "string"
                },
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

## Configuration

```yaml
...
map:
  plugin: "dc_measurements/Map"
  topic_output: "/dc/measurement/map"
  topic: "/map"
  save_path: "map/%Y-%m-%dT%H:%M:%S"
  save_map_timeout: 0.2
  quiet: true
  remote_keys: ["s3"]
```

## Example output

A 4x4 test grid, captured without `remote_keys` configured (so no `remote_paths`):

```json
{
  "flattened": false,
  "height": 4,
  "local_paths": {
    "pgm": "/tmp/dc_capture/map_out/2026-09-05T23:46:20.pgm",
    "png": "/tmp/dc_capture/map_out/2026-09-05T23:46:20.png",
    "yaml": "/tmp/dc_capture/map_out/2026-09-05T23:46:20.yaml"
  },
  "name": "map",
  "nested": false,
  "origin": {
    "x": 0.0,
    "y": 0.0
  },
  "resolution": 0.05000000074505806,
  "run_id": "169",
  "width": 4
}
```
