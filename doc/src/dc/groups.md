# Groups

## Description
Each measurement is part of a group. This allows measurements to be grouped together later on. For example, having the measurements cpu and position belonging to the same group, data will be published together on `/dc/group/my_group`

```json
{
    "cpu": ...,
    "position": ...,
}
```

Currently, this node is written in python. The main reason for it is that we don't know how to allocate and pass different amount of variables to the TimeSynchronizer. In python it is quite simple, but not so in C++.

## Node parameters

| Parameter | Description      | Type      | Default |
| --------- | ---------------- | --------- | ------- |
| groups    | Groups to enable | list[str] | N/A     |

## Group parameters

| Parameter    | Description                                                                         | Type      | Default             |
| ------------ | ----------------------------------------------------------------------------------- | --------- | ------------------- |
| inputs       | Name of the input topics to group                                                   | list[str] | N/A                 |
| output       | Output topic to send the data to                                                    | str       | "/dc/group/{group}" |
| sync_delay   | Delay to wait during all subscriber data need to reach before being published again | float     | 5.0                 |
| group_key    | Dictionary key under which data is grouped                                          | str       | {group_name}        |
| exclude_keys | List of keys to exclude from the published data. Data depth is separated by a dot   | list[str] | N/A                 |
| tags         | Destination names, used to know where data will be forwarded to                     | list[str] | N/A                 |
| nested_data  | Whether measurements are nested dictionaries or flat                                | bool      | false               |

## Example

```yaml
group_server:
  ros__parameters:
    groups: ["map", "memory_uptime"]
    memory_cpu:
      inputs: ["/dc/measurement/memory", "/dc/measurement/cpu"]
      output: "/dc/group/memory_cpu"
      sync_delay: 5.0
      group_key: "memory_cpu"
      tags: ["flb_pgsql"]
    memory_uptime:
      inputs: ["/dc/measurement/memory", "/dc/measurement/uptime"]
      output: "/dc/group/memory_uptime"
      sync_delay: 5.0
      group_key: "memory_uptime"
      tags: ["flb_stdout"]
    cameras:
      inputs: ["/dc/measurement/camera"]
      output: "/dc/group/cameras"
      sync_delay: 5.0
      group_key: "cameras"
      tags: ["flb_minio", "flb_stdout"]
    map:
      inputs: ["/dc/measurement/map"]
      output: "/dc/group/map"
      sync_delay: 5.0
      group_key: "map"
      tags: ["flb_minio", "flb_stdout"]
```
