# List integer equal

## Description

Compare JSON key value to the value passed in parameter and returns true if equal.

## Parameters

| Parameter         | Description                                                            | Type        | Default         |
| ----------------- | ---------------------------------------------------------------------- | ----------- | --------------- |
| **key**           | JSON key where value is located, separate nested dictionary with **/** | str         | N/A (Mandatory) |
| **value**         | Value to which compare the JSON value                                  | list\[int\] | N/A (Mandatory) |
| **order_matters** | If true, will compare taking account of the order                      | bool        | true            |

## Configuration

A custom node publishes a [StringStamped](../measurements/string_stamped.md) Record with a
`bay_status_codes: [int, int, int]` array, one per charging bay. Only forward it once a
code strays from the nominal all-`200` baseline:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["bays_nominal"]
    bays_nominal:
      plugin: "dc_conditions/ListIntegerEqual"
      key: "bay_status_codes"
      value: [200, 200, 200]
      order_matters: true
    bay_status_codes:
      plugin: "dc_measurements/StringStamped"
      if_none_conditions: ["bays_nominal"]
      topic_output: "/dc/measurement/bay_status_codes"
      topic: "/bay_status_codes_raw"
      enable_validator: false
```
