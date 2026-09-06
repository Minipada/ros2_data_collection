# List double equal

## Description

Compare JSON key value to the value passed in parameter and returns true if equal.

## Parameters

| Parameter         | Description                                                            | Type          | Default         |
| ----------------- | ---------------------------------------------------------------------- | ------------- | --------------- |
| **key**           | JSON key where value is located, separate nested dictionary with **/** | str           | N/A (Mandatory) |
| **value**         | Value to which compare the JSON value                                  | list\[float\] | N/A (Mandatory) |
| **order_matters** | If true, will compare taking account of the order                      | bool          | true            |

## Configuration

A custom node publishes a [StringStamped](../measurements/string_stamped.md) Record with a
`zone_temperatures: [float, float, float]` array. Only forward it once the readings move
away from the expected baseline:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["zones_at_baseline"]
    zones_at_baseline:
      plugin: "dc_conditions/ListDoubleEqual"
      key: "zone_temperatures"
      value: [21.0, 21.0, 21.0]
      order_matters: true
    zone_temperatures:
      plugin: "dc_measurements/StringStamped"
      if_none_conditions: ["zones_at_baseline"]
      topic_output: "/dc/measurement/zone_temperatures"
      topic: "/zone_temperatures_raw"
      enable_validator: false
```
