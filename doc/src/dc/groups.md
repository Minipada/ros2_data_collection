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
| group_key          | Dictionary key under which data is grouped                                          | str         | {group_name}        |
| exclude_keys       | List of keys to exclude from the published data. Data depth is separated by a dot   | list\[str\] | N/A                 |
| nested_data        | Whether measurements are nested dictionaries or flat                                | bool        | false               |
| include_group_name | Include group name in the JSON as key="name" and value=<group_key>                  | bool        | true                |

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
