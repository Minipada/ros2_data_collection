# List string equal

## Description

Compare JSON key value to the value passed in parameter and returns true if equal.

## Parameters

| Parameter         | Description                                                            | Type | Default         |
| ----------------- | ---------------------------------------------------------------------- | ---- | --------------- |
| **key**           | JSON key where value is located, separate nested dictionary with **/** | str  | N/A (Mandatory) |
| **value**         | Value to which compare the JSON value                                  | list\[str\] | N/A (Mandatory) |
| **order_matters** | If true, will compare taking account of the order                      | bool | true            |

## Configuration

[Fast DDS statistics](../measurements/fastdds_stats.md) reports the discovered `hosts`
list. Only publish it once that list no longer matches the expected single-host
fingerprint — flags an unexpected extra participant joining the DDS graph:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["known_hosts"]
    known_hosts:
      plugin: "dc_conditions/ListStringEqual"
      key: "hosts"
      value: ["d:14058711922191368192"]
      order_matters: false
    fastdds_stats:
      plugin: "dc_measurements/FastddsStats"
      if_none_conditions: ["known_hosts"]
      topic_output: "/dc/measurement/fastdds_stats"
```
