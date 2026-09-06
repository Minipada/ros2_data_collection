# Integer inferior

## Description

Compare JSON key value to the value passed in parameter and returns true if inferior.

## Parameters

| Parameter | Description                                                            | Type | Default         |
| --------- | ---------------------------------------------------------------------- | ---- | --------------- |
| **key**   | JSON key where value is located, separate nested dictionary with **/** | str  | N/A (Mandatory) |
| **value** | Value to which compare the JSON value                                  | int  | N/A (Mandatory) |
| **include_value** | Use `<=` (true) instead of `<` (false)                        | bool | true            |

## Configuration

Only publish [CPU](../measurements/cpu.md) while few processes crossed `cpu_min`
(`processes <= 5`) — a busy system already gets enough detail from the `sorted` list
without this extra Record:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["few_hot_processes"]
    few_hot_processes:
      plugin: "dc_conditions/IntegerInferior"
      key: "processes"
      value: 5
      include_value: true
    cpu:
      plugin: "dc_measurements/Cpu"
      if_all_conditions: ["few_hot_processes"]
      topic_output: "/dc/measurement/cpu"
```
