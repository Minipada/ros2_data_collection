# Uptime

## Description
Time since when the robot PC has been on.

## Parameters

This Measurement has no parameters beyond the [common Plugin parameters](../measurements.md#plugin-parameters).

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Uptime",
    "description": "Time the system has been up",
    "properties": {
        "time": {
            "description": "Time the system has been up",
            "type": "integer",
            "minimum": 0
        }
    },
    "type": "object"
}
```

## Configuration

```yaml
...
uptime:
  plugin: "dc_measurements/Uptime"
  topic_output: "/dc/measurement/uptime"
```

## Example output

```json
{
  "flattened": false,
  "name": "uptime",
  "nested": false,
  "run_id": "169",
  "time": 1783191
}
```
