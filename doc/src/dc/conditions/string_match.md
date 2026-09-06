# String match

## Description

Compare JSON key value to the value passed in parameter and returns true if match.

## Parameters

| Parameter | Description                                                            | Type | Default         |
| --------- | ---------------------------------------------------------------------- | ---- | --------------- |
| **key**   | JSON key where value is located, separate nested dictionary with **/** | str  | N/A (Mandatory) |
| **regex** | Regex to which compare the JSON value to                               | str  | N/A (Mandatory) |

## Configuration

Only publish [Battery](../measurements/battery.md) while it's actively charging or full:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["actively_charging"]
    actively_charging:
      plugin: "dc_conditions/StringMatch"
      key: "power_supply_status"
      regex: "charging|full"
    battery:
      plugin: "dc_measurements/Battery"
      if_all_conditions: ["actively_charging"]
      topic_output: "/dc/measurement/battery"
```
