# Group memory and uptime

This demo will introduce the group node. It subscribes to multiple nodes and group for each its data and republishes on a new topic.

Let's run it:

```bash
$ ros2 launch dc_demos group_memory_uptime_stdout.launch.py
```

```
[component_container_isolated-1] [{"memory":{"used":71.67569732666016},"date":1677673405.921659,"uptime":{"time":96894},"run_id":"221"}]
[component_container_isolated-1] [{"memory":{"used":71.70790100097656},"date":1677673408.868847,"uptime":{"time":96897},"run_id":"221"}]
[component_container_isolated-1] [{"memory":{"used":71.72582244873047},"date":1677673411.869299,"uptime":{"time":96900},"run_id":"221"}]
[component_container_isolated-1] [{"memory":{"used":71.86643218994141},"date":1677673414.869292,"uptime":{"time":96903},"run_id":"221"}]
[component_container_isolated-1] [{"memory":{"used":71.82553863525391},"date":1677673417.869207,"uptime":{"time":96906},"run_id":"221"}]
```

This launchfile is a wrapper of [dc_bringup/launch/bringup.launch.py](https://github.com/Minipada/ros2_data_collection/blob/humble/dc_bringup/launch/dc_bringup.launch.py) which loads a [custom yaml configuration](https://github.com/Minipada/ros2_data_collection/blob/humble/dc_demos/params/group_memory_uptime_stdout.yaml)

```admonish info
Note that here the group node is started. It is one parameter in the launchfile to enable it. In the [uptime demo](./uptime_stdout.md), it is disabled by default because it is not used.
```

## Configuration

### Measurement
We collect data from 2 plugins: memory and uptime. The first every second and the latter every 3.

```yaml
measurement_server:
  ros__parameters:
    measurement_plugins: ["memory", "uptime"]
    memory:
      plugin: "dc_measurements/Memory"
      group_key: "memory"
      topic_output: "/dc/measurement/memory"
      polling_interval: 1000
    uptime:
      plugin: "dc_measurements/Uptime"
      group_key: "uptime"
      topic_output: "/dc/measurement/uptime"
      polling_interval: 3000
```

Data is now published on 2 ROS topics: /dc/measurement/uptime and /dc/measurement/memory.

The `group_key` mentioned will be used by the group node to assign a key in the new dictionary

### Group

This create a memory_uptime group, subscribes to `/dc/measurement/memory` and `/dc/measurement/uptime` topics and republish the result on `/dc/group/memory_uptime`. The sync_delay allows to wait in a 5 seconds window timeframe the data from each topic before throwing away the data if one topic does not publish it

```yaml
group_server:
  ros__parameters:
    groups: ["memory_uptime"]
    memory_uptime:
      inputs: ["/dc/measurement/memory", "/dc/measurement/uptime"]
      output: "/dc/group/memory_uptime"
      sync_delay: 5.0
      group_key: "memory_uptime"
      tags: ["flb_stdout"]
```

### Destination

Here, we only subscribe to the `/dc/group/memory_uptime` topic

```yaml
    flb_stdout:
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/group/memory_uptime"]
```

## Console output

In the terminal, you can see the result, published every 3 seconds (see the date field), which the is timeframe defined by sync_delay and the maximum polling_interval of the measurements.

Finally, note the new dictionary uses the key defined in the `group_key` measurement_server plugin configuration. They are transferred through the ROS message.

```
[component_container_isolated-1] [{"memory":{"used":71.67569732666016},"date":1677673405.921659,"uptime":{"time":96894},"run_id":"221"}]
[component_container_isolated-1] [{"memory":{"used":71.70790100097656},"date":1677673408.868847,"uptime":{"time":96897},"run_id":"221"}]
[component_container_isolated-1] [{"memory":{"used":71.72582244873047},"date":1677673411.869299,"uptime":{"time":96900},"run_id":"221"}]
[component_container_isolated-1] [{"memory":{"used":71.86643218994141},"date":1677673414.869292,"uptime":{"time":96903},"run_id":"221"}]
[component_container_isolated-1] [{"memory":{"used":71.82553863525391},"date":1677673417.869207,"uptime":{"time":96906},"run_id":"221"}]
```
