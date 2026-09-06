# Integer equal

## Description

Compare JSON key value to the value passed in parameter and returns true if equal.

## Parameters

| Parameter | Description                                                            | Type | Default         |
| --------- | ---------------------------------------------------------------------- | ---- | --------------- |
| **key**   | JSON key where value is located, separate nested dictionary with **/** | str  | N/A (Mandatory) |
| **value** | Value to which compare the JSON value                                  | int  | N/A (Mandatory) |

## Configuration

Only publish [Battery](../measurements/battery.md) while it's still on its very first
charge cycle (`completed_cycles: 0`) — flags a freshly installed pack:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["brand_new_pack"]
    brand_new_pack:
      plugin: "dc_conditions/IntegerEqual"
      key: "completed_cycles"
      value: 0
    battery:
      plugin: "dc_measurements/Battery"
      if_all_conditions: ["brand_new_pack"]
      topic_output: "/dc/measurement/battery"
```
