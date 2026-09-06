# Bool equal

## Description

Compare JSON key value to the value passed in parameter and returns true if equal.

## Parameters

| Parameter | Description                                                            | Type | Default         |
| --------- | ---------------------------------------------------------------------- | ---- | --------------- |
| **key**   | JSON key where value is located, separate nested dictionary with **/** | str  | N/A (Mandatory) |
| **value** | Value to which compare the JSON value                                  | bool | N/A (Mandatory) |

## Configuration

A Condition is evaluated against the Record of the Measurement it gates, so `key` names one
of that same Measurement's own fields. Here, only forward a
[TCP Health](../measurements/tcp_health.md) Record when the check actually failed —
suppressing the steady stream of `active: true` polls and keeping only outage alerts:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["endpoint_down"]
    endpoint_down:
      plugin: "dc_conditions/BoolEqual"
      key: "active"
      value: false
    rustfs_health:
      plugin: "dc_measurements/TCPHealth"
      if_all_conditions: ["endpoint_down"]
      topic_output: "/dc/measurement/rustfs_health"
      host: "127.0.0.1"
      port: 9000
      name: "rustfs_api"
```
