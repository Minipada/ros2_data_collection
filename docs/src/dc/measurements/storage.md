# Storage

## Description

Collect storage information on a directory.

## Parameters

| Parameter | Description                                                             | Type | Default         |
| --------- | ----------------------------------------------------------------------- | ---- | --------------- |
| **path**  | Absolute path to the directory to inspect, expect environment variables | str  | N/A (Mandatory) |

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Storage",
    "description": "Storage information of a directory",
    "properties": {
        "free_percent": {
            "description": "Free space on the filesystem, in percent",
            "type": "number",
            "minimum": 0
        },
        "free": {
            "description": "Free space on the filesystem, in bytes",
            "type": "integer",
            "minimum": 0
        },
        "capacity": {
            "description": "Total size of the filesystem, in bytes",
            "type": "integer",
            "minimum": 0
        }
    },
    "type": "object"
}
```

## Measurement configuration

```yaml
...
storage_home:
  plugin: "dc_measurements/Storage"
  topic_output: "/dc/measurement/storage_home"
  tags: ["flb_stdout"]
  path: "$HOME"
```
