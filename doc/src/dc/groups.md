# Groups

## Description

A **Group** merges the Records of several Measurements into one Record, based on time
proximity. For example, grouping the `cpu` and `position` Measurements publishes a single
merged Record on `/dc/group/my_group`:

```json
{
    "cpu": ...,
    "position": ...,
}
```

A merged Record is an ordinary Record from there on: it carries the Tag derived from its
output topic, and a Destination receives it by listing that topic in `inputs`.

Envelope fields do not get nested under a member's key: a member's `tags` is replaced by the
Group's own, and its `incident_id` — the id a Measurement stamps on a Record it released as
part of an [incident](./measurements.md) — is lifted to the merged Record's top level, where
a `postgres` Destination has a column for it. Buried under `<group_key>.incident_id` it would
simply be dropped by that sink.

```admonish info
The Group node is written in Python: allocating and passing a variable number of inputs to
the `ApproximateTimeSynchronizer` is straightforward there and awkward in C++.
```

```admonish warning title="tags no longer selects a Destination"
As with Measurements, a Group's DC 1.x `tags` parameter no longer routes anything —
remove it and list the Group's `output` topic in the receiving Destination's `inputs`.
See the [migration guide](./migration.md#tags-what-changed).
```

## Node parameters

| Parameter | Description      | Type        | Default |
| --------- | ---------------- | ----------- | ------- |
| groups    | Groups to enable | list\[str\] | N/A     |

## Group parameters

| Parameter          | Description                                                                         | Type        | Default             |
| ------------------ | ----------------------------------------------------------------------------------- | ----------- | ------------------- |
| inputs             | Name of the input topics to group                                                   | list\[str\] | N/A                 |
| output             | Output topic to send the data to                                                    | str         | "/dc/group/{group}" |
| sync_delay         | Delay to wait during all subscriber data need to reach before being published again | float       | 5.0                 |
| sync_timeout       | Seconds an incomplete set may wait before `on_sync_timeout` applies. 0.0 disables it | float       | 0.0                 |
| on_sync_timeout    | What to do with an incomplete set once `sync_timeout` elapses: `drop`/`emit_partial` | str         | "drop"              |
| sync_timeout_log_throttle | Seconds between two sync-timeout warnings for this group. 0.0 logs every one  | float       | 60.0                |
| group_key          | Dictionary key under which data is grouped                                          | str         | {group_name}        |
| exclude_keys       | List of keys to exclude from the published data. Data depth is separated by a dot   | list\[str\] | N/A                 |
| nested_data        | Whether measurements are nested dictionaries or flat                                | bool        | false               |
| include_group_name | Include group name in the JSON as key="name" and value=<group_key>                  | bool        | true                |

## Incomplete sets

A Group only publishes once *every* one of its `inputs` has produced a Record close enough
in time. If one Measurement stops publishing — a camera unplugged, a plugin crashed, a
`gate_condition` that never opens — the whole Group goes silent, and the Records of the
inputs that *are* working never reach a Destination.

`sync_timeout` puts a deadline on that wait. It is measured from the arrival of the first
Record of a set, and it is a different thing from `sync_delay`: `sync_delay` is the
synchroniser's slop, the maximum spread between the *timestamps* of Records that may be
merged together, whereas `sync_timeout` is real time spent waiting for a set to complete.

| `sync_timeout` | `on_sync_timeout` | Behaviour                                                                                        |
| -------------- | ----------------- | ------------------------------------------------------------------------------------------------ |
| `0.0`          | (ignored)         | No deadline. The Group waits indefinitely and the synchroniser evicts stale Records on its own    |
| `> 0.0`        | `drop`            | The incomplete set is discarded when the deadline elapses. Nothing is published, a warning is logged |
| `> 0.0`        | `emit_partial`    | The Records that did arrive are published as one **partial Record**, then the set is discarded    |

`drop` is the default, so a Group that sets nothing new behaves exactly as it did before.
`emit_partial` needs a positive `sync_timeout` to have a deadline to fire on; configured
without one, the Group logs an error and falls back to `drop`.

```admonish info
The deadline is on the set as a whole, not on each input: it starts with the first Record
of a set and later Records do not push it back. After a set times out, the Group forgets
every Record it was holding, so a Record already published in a partial set is never
merged a second time when its late partner finally arrives.
```

### Warnings and throttling

Every timeout logs a warning naming the group, the deadline it missed, and the input
topics that produced nothing — including under `drop`, where the warning is the only trace
left since nothing is published:

```text
[WARN] [group_server]: Group 'memory_cpu': no complete set after 10.0s, dropping the
incomplete set (no Record from: ['/dc/measurement/memory'])
```

A Measurement that dies stays dead, so the group keeps timing out and an unthrottled
warning would repeat every `sync_timeout` for as long as the robot runs.
`sync_timeout_log_throttle` caps this at one warning per group per window (60s by default);
set it to `0.0` to log every timeout. The first timeout after a quiet window always logs
immediately, and reports how many warnings the throttle swallowed since the last one:

```text
[WARN] [group_server]: Group 'memory_cpu': no complete set after 10.0s, dropping the
incomplete set (no Record from: ['/dc/measurement/memory']) [+5 more in the last 60.0s]
```

The throttle is per group, so a permanently broken group does not silence the warnings of a
healthy one that starts failing. Only the *log line* is rate-limited — under `emit_partial`
every timeout still publishes its partial Record, throttled or not.

### What a partial Record looks like

A partial Record has the same shape as a complete one, minus the inputs that did not
report, plus two keys that mark it as partial:

```json
{
    "cpu": { "used": 12.0 },
    "partial": true,
    "missing_inputs": ["/dc/measurement/memory"],
    "tags": [""],
    "name": "memory_cpu"
}
```

- **A missing input's key is omitted, not present-and-null.** The Group node knows a
  missing input's *topic* and nothing else: a Record's `group_key` and its fields both
  travel inside the Record itself, so there is no correctly shaped null placeholder the
  Group could invent for a Measurement that never published. Consumers with a flat column
  schema — the `postgres` sink maps a Record's top-level keys onto existing columns — see
  those columns stay `NULL` for that row, the same as any other column a Record does not
  populate.
- **`partial` and `missing_inputs` appear only on partial Records.** A complete Record is
  byte-for-byte what the Group published before this option existed, so nothing downstream
  has to change to keep consuming complete Records. Treat the absence of `partial` as
  `false`.
- `missing_inputs` lists the input *topics* that contributed no Record, in `inputs` order.
- `tags`, `name` and `plugins` behave as they do on a complete Record — `plugins` only
  lists the plugins that actually reported.
- `incident_id` behaves the same way: if any member that *did* arrive was released as part
  of an [incident](./measurements.md), the merged Record carries its `incident_id`, so a
  partial Record of an incident is still queryable as one.

```admonish warning title="Add the columns before charting a partial Record"
Vector's `postgres` sink silently drops top-level keys that have no matching column, so
`partial` and `missing_inputs` are discarded unless you add `partial boolean` and
`missing_inputs jsonb` to the destination table yourself.
```

## Example

```yaml
group_server:
  ros__parameters:
    groups: ["memory_cpu", "memory_uptime", "cameras", "map"]
    memory_cpu:
      inputs: ["/dc/measurement/memory", "/dc/measurement/cpu"]
      output: "/dc/group/memory_cpu"
      sync_delay: 5.0
      group_key: "memory_cpu"
    memory_uptime:
      inputs: ["/dc/measurement/memory", "/dc/measurement/uptime"]
      output: "/dc/group/memory_uptime"
      sync_delay: 5.0
      # Publish whatever arrived rather than nothing if one of the two is 10s late
      sync_timeout: 10.0
      on_sync_timeout: "emit_partial"
      group_key: "memory_uptime"
    cameras:
      inputs: ["/dc/measurement/camera"]
      output: "/dc/group/cameras"
      sync_delay: 5.0
      group_key: "cameras"
    map:
      inputs: ["/dc/measurement/map"]
      output: "/dc/group/map"
      sync_delay: 5.0
      group_key: "map"
```
