# List bool equal

## Description

Compare JSON key value to the value passed in parameter and returns true if equal.

## Parameters

| Parameter         | Description                                                            | Type | Default         |
| ----------------- | ---------------------------------------------------------------------- | ---- | --------------- |
| **key**           | JSON key where value is located, separate nested dictionary with **/** | str  | N/A (Mandatory) |
| **value**         | Value to which compare the JSON value                                  | list\[bool\] | N/A (Mandatory) |
| **order_matters** | If true, will compare taking account of the order                      | bool | true            |

## Configuration

A custom node publishes a [StringStamped](../measurements/string_stamped.md) Record with a
`door_sensors: [bool, bool, bool]` array, one per bay. Only forward it once the pattern
deviates from the all-closed baseline:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["doors_all_closed"]
    doors_all_closed:
      plugin: "dc_conditions/ListBoolEqual"
      key: "door_sensors"
      value: [true, true, true]
      order_matters: true
    door_sensors:
      plugin: "dc_measurements/StringStamped"
      if_none_conditions: ["doors_all_closed"]
      topic_output: "/dc/measurement/door_sensors"
      topic: "/door_sensors_raw"
      enable_validator: false
```
