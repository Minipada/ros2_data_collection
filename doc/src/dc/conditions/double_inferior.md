# Double inferior

## Description

Compare JSON key value to the value passed in parameter and returns true if inferior.

## Parameters

| Parameter | Description                                                            | Type  | Default         |
| --------- | ---------------------------------------------------------------------- | ----- | --------------- |
| **key**   | JSON key where value is located, separate nested dictionary with **/** | str   | N/A (Mandatory) |
| **value** | Value to which compare the JSON value                                  | float | N/A (Mandatory) |
| **include_value** | Use `<=` (true) instead of `<` (false)                        | bool  | true            |

## Configuration

Paired with [Double superior](./double_superior.md) to only collect
[Distance traveled](../measurements/distance_traveled.md) once it's outside a
dead-band — filters out both "parked" noise and implausibly large jumps
(`dc_demos/params/tb3_simulation_pgsql_minio.yaml`):

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["min_distance_traveled", "max_distance_traveled"]
    min_distance_traveled:
      plugin: "dc_conditions/DoubleSuperior"
      key: "distance_traveled"
      value: 0.01
      include_value: true
    max_distance_traveled:
      plugin: "dc_conditions/DoubleInferior"
      key: "distance_traveled"
      value: 2.0
      include_value: true
    distance_traveled:
      plugin: "dc_measurements/DistanceTraveled"
      if_all_conditions: ["min_distance_traveled", "max_distance_traveled"]
      topic_output: "/dc/measurement/distance_traveled"
```
