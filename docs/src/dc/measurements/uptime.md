# Uptime

## Description
Time since when the robot PC has been on.

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

## Measurement configuration

```yaml
...
uptime:
  plugin: "dc_measurements/Uptime"
  topic_output: "/dc/measurement/uptime"
  tags: ["flb_stdout"]
```
