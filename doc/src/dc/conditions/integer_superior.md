# Integer superior

## Description

Compare JSON key value to the value passed in parameter and returns true if superior.

## Parameters

| Parameter | Description                                                            | Type | Default         |
| --------- | ---------------------------------------------------------------------- | ---- | --------------- |
| **key**   | JSON key where value is located, separate nested dictionary with **/** | str  | N/A (Mandatory) |
| **value** | Value to which compare the JSON value                                  | int  | N/A (Mandatory) |
| **include_value** | Use `>=` (true) instead of `>` (false)                        | bool | true            |

## Configuration

Only publish [Battery](../measurements/battery.md) once a pack has crossed 500
completed cycles — flags an aging pack worth scheduling for replacement:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["pack_aging"]
    pack_aging:
      plugin: "dc_conditions/IntegerSuperior"
      key: "completed_cycles"
      value: 500
      include_value: true
    battery:
      plugin: "dc_measurements/Battery"
      if_all_conditions: ["pack_aging"]
      topic_output: "/dc/measurement/battery"
```
