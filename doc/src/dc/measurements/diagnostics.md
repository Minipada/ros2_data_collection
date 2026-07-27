# Diagnostics

## Description
Subscribes to `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) and converts matching
`DiagnosticStatus` entries into a Record, so hardware/driver health reaches Destinations and
dashboards like any other Measurement. Each `DiagnosticStatus.values` key/value pair is preserved
as-is in the Record rather than being flattened into the message string.

`/diagnostics` is typically high-volume and mostly unchanging, so `level_threshold` and `names`
are provided to shrink what gets collected. Pair this Measurement with the `same_as_previous`
condition (`if_none_conditions`) to also skip republishing when nothing has changed since the
previous collection.

## Parameters

| Parameter            | Description                                                                | Type      | Default            |
| --------------------- | --------------------------------------------------------------------------- | --------- | ------------------- |
| **topic**             | Topic to subscribe to for diagnostics                                       | str       | "/diagnostics" (Optional) |
| **level_threshold**   | Minimum status level to collect: `"OK"`, `"WARN"`, `"ERROR"`, or `"STALE"`   | str       | "OK" (Optional)     |
| **names**             | Allowlist of `DiagnosticStatus.name` values to collect; empty collects all  | list[str] | [] (Optional)       |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Diagnostics",
  "description": "Diagnostic statuses collected from /diagnostics",
  "properties": {
    "statuses": {
      "description": "Diagnostic statuses matching the configured level threshold and name allowlist",
      "type": "array",
      "items": {
        "title": "DiagnosticStatus",
        "description": "A single diagnostic_msgs/DiagnosticStatus entry",
        "properties": {
          "name": {
            "description": "Reporting component name",
            "type": "string"
          },
          "message": {
            "description": "Human-readable status summary",
            "type": "string"
          },
          "hardware_id": {
            "description": "Hardware identifier",
            "type": "string"
          },
          "level": {
            "description": "Status level: 0=OK, 1=WARN, 2=ERROR, 3=STALE",
            "type": "integer",
            "minimum": 0,
            "maximum": 3
          },
          "values": {
            "description": "Key/value pairs reported by the status, preserved as-is",
            "type": "object"
          }
        },
        "required": ["name", "message", "level", "values"],
        "type": "object"
      }
    }
  },
  "required": ["statuses"],
  "type": "object"
}
```

## Measurement configuration

```yaml
...
diagnostics:
  plugin: "dc_measurements/Diagnostics"
  topic_output: "/dc/measurement/diagnostics"
  tags: ["console"]
  level_threshold: "WARN"
  names: ["motor_driver", "battery"]
  if_none_conditions: ["diagnostics_unchanged"]

diagnostics_unchanged:
  plugin: "dc_conditions/SameAsPrevious"
  keys: []
  exclude: []
```
