# Compare

## Description

Compare the value of a JSON key against a configured operand and activate while the comparison
holds. One plugin covers the whole family: `comparison` picks the operator and the type you give
`value` picks the operand type (the two axes thirteen former per-operator plugins each
hard-coded).

## Parameters

| Parameter | Description | Type | Default |
| --------- | -------------------------------------------------------------------------- | ----------------------- | --------------- |
| **key** | JSON key where value is located, separate nested dictionary with **/** | str | N/A (Mandatory) |
| **comparison** | Operator: `eq`, `ne`, `gt`, `ge`, `lt`, `le`, `match` or `exists` | str | N/A (Mandatory) |
| **value** | Operand to compare the JSON value against; its type selects the operand type: `bool`, `int`, `float`, `str`, `list[bool]`, `list[int]`, `list[float]` or `list[str]` | any of those | N/A (Mandatory, except for `match` and `exists`) |
| **regex** | Regex the JSON value must fully match | str | N/A (Mandatory for `match`) |
| **order_matters** | For list operands: compare element order (`true`) or as unordered multisets (`false`) | bool | true |

The JSON comparison is **type-strict**: a Record field written as `5` is an integer and never
matches a `float` operand (and `5.0` never matches an `int` one). `gt`/`ge`/`lt`/`le` need an
`int` or `float` operand; `ne` inverts `eq` for every operand type.

## Configuration

Only forward a [TCP Health](../measurements/tcp_health.md) Record when the check actually
failed — suppressing the steady stream of `active: true` polls and keeping only outage alerts:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["endpoint_down"]
    endpoint_down:
      plugin: "dc_conditions/Compare"
      key: "active"
      value: false
      comparison: "eq"
    rustfs_health:
      plugin: "dc_measurements/TCPHealth"
      if_all_conditions: ["endpoint_down"]
      topic_output: "/dc/measurement/rustfs_health"
      host: "127.0.0.1"
      port: 9000
      name: "rustfs_api"
```

A dead-band on [Distance traveled](../measurements/distance_traveled.md) — filters out both
"parked" noise and implausibly large jumps (`dc_demos/params/tb3_simulation_pgsql_minio.yaml`):

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["min_distance_traveled", "max_distance_traveled"]
    min_distance_traveled:
      plugin: "dc_conditions/Compare"
      key: "distance_traveled"
      value: 0.01
      comparison: "ge"
    max_distance_traveled:
      plugin: "dc_conditions/Compare"
      key: "distance_traveled"
      value: 2.0
      comparison: "le"
    distance_traveled:
      plugin: "dc_measurements/DistanceTraveled"
      if_all_conditions: ["min_distance_traveled", "max_distance_traveled"]
      topic_output: "/dc/measurement/distance_traveled"
```

Only publish [Fast DDS statistics](../measurements/fastdds_stats.md) once its discovered
`hosts` list no longer matches the expected single-host fingerprint — flags an unexpected extra
participant joining the DDS graph:

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["known_hosts"]
    known_hosts:
      plugin: "dc_conditions/Compare"
      key: "hosts"
      value: ["d:14058711922191368192"]
      comparison: "eq"
      order_matters: false
    fastdds_stats:
      plugin: "dc_measurements/FastddsStats"
      if_none_conditions: ["known_hosts"]
      topic_output: "/dc/measurement/fastdds_stats"
```

Only collect a [Battery](../measurements/battery.md) Record while it is actively charging
(`match` uses a full `regex` match, and `exists` tests only that `key` is present, taking no
`value`):

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["actively_charging", "inspected_exists"]
    actively_charging:
      plugin: "dc_conditions/Compare"
      key: "power_supply_status"
      comparison: "match"
      regex: "charging|full"
    inspected_exists:
      plugin: "dc_conditions/Compare"
      key: "inspected"
      comparison: "exists"
    battery:
      plugin: "dc_measurements/Battery"
      if_all_conditions: ["actively_charging", "inspected_exists"]
      topic_output: "/dc/measurement/battery"
```

## Migration from the former per-operator plugins

| Former plugin | `comparison` | `value` type | Notes |
| ------------------------------- | ------------ | ------------ | ------------------------------ |
| `dc_conditions/BoolEqual` | `eq` | `bool` | |
| `dc_conditions/DoubleEqual` | `eq` | `float` | |
| `dc_conditions/IntegerEqual` | `eq` | `int` | |
| `dc_conditions/DoubleInferior` | `le` or `lt` | `float` | `le` was `include_value: true`, `lt` `false` |
| `dc_conditions/IntegerInferior` | `le` or `lt` | `int` | same `include_value` mapping |
| `dc_conditions/DoubleSuperior` | `ge` or `gt` | `float` | `ge` was `include_value: true`, `gt` `false` |
| `dc_conditions/IntegerSuperior` | `ge` or `gt` | `int` | same `include_value` mapping |
| `dc_conditions/ListBoolEqual` | `eq` | `list[bool]` | `order_matters` unchanged |
| `dc_conditions/ListDoubleEqual` | `eq` | `list[float]` | `order_matters` unchanged |
| `dc_conditions/ListIntegerEqual` | `eq` | `list[int]` | `order_matters` unchanged |
| `dc_conditions/ListStringEqual` | `eq` | `list[str]` | `order_matters` unchanged |
| `dc_conditions/StringMatch` | `match` | none | `regex` parameter unchanged |
| `dc_conditions/Exist` | `exists` | none | `key` only |

`ne` (value differs) and scalar `str` equality are new with `Compare`.
