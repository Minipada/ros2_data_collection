# Same as previous

## Description

Compare JSON key value to the value passed in parameter and returns true if match.

## Parameters

| Parameter   | Description                                                               | Type        | Default         |
| ----------- | ------------------------------------------------------------------------- | ----------- | --------------- |
| **keys**    | JSON keys where values are located, separate nested dictionary with **/** | list\[str\] | N/A (Mandatory) |
| **exclude** | JSON keys to exclude in comparison                                        | list\[str\] | N/A (Mandatory) |

## Configuration

Skip republishing [Diagnostics](../measurements/diagnostics.md) when nothing has changed
since the last collection — `/diagnostics` is typically high-volume and mostly unchanging:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["diagnostics_unchanged"]
    diagnostics_unchanged:
      plugin: "dc_conditions/SameAsPrevious"
      keys: []
      exclude: []
    diagnostics:
      plugin: "dc_measurements/Diagnostics"
      if_none_conditions: ["diagnostics_unchanged"]
      topic_output: "/dc/measurement/diagnostics"
      level_threshold: "WARN"
      names: ["motor_driver", "battery"]
```
