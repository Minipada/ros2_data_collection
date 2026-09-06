# Group memory and uptime

This demo will introduce the group node. It subscribes to multiple nodes and group for each its data and republishes on a new topic.

Let's run it:

```bash
ros2 launch dc_demos group_memory_uptime_stdout.launch.py
```

```
[dc_bridge-3] {"date":1788476662.509069,"host":"127.0.0.1","memory":{"flattened":false,"name":"memory","nested":false,"run_id":"169","used":95.77983856201172},"name":"memory_uptime","source_type":"fluent","tag":"dc.group.memory_uptime","tags":[""],"timestamp":"2026-09-03T23:04:22.509068935Z","uptime":{"flattened":false,"name":"uptime","nested":false,"run_id":"169","time":1609047}}
[dc_bridge-3] {"date":1788476665.5074596,"host":"127.0.0.1","memory":{"flattened":false,"name":"memory","nested":false,"run_id":"169","used":95.87796020507812},"name":"memory_uptime","source_type":"fluent","tag":"dc.group.memory_uptime","tags":[""],"timestamp":"2026-09-03T23:04:25.507459531Z","uptime":{"flattened":false,"name":"uptime","nested":false,"run_id":"169","time":1609050}}
[dc_bridge-3] {"date":1788476668.5055394,"host":"127.0.0.1","memory":{"flattened":false,"name":"memory","nested":false,"run_id":"169","used":96.04102325439453},"name":"memory_uptime","source_type":"fluent","tag":"dc.group.memory_uptime","tags":[""],"timestamp":"2026-09-03T23:04:28.505539347Z","uptime":{"flattened":false,"name":"uptime","nested":false,"run_id":"169","time":1609053}}
```

This launchfile is a wrapper of [dc_bringup/launch/dc_bringup.launch.py](https://github.com/Minipada/ros2_data_collection/blob/jazzy/dc_bringup/launch/dc_bringup.launch.py) which loads a [custom yaml configuration](https://github.com/Minipada/ros2_data_collection/blob/jazzy/dc_demos/params/group_memory_uptime_stdout.yaml)

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
```

**include_group_name (Optional, default `true`)**: Includes the name of the group in the JSON as a top-level `name` field — visible in the console output below — which makes it easier later on to fetch the data from your API. Left at its default here.

You can also notice that the group also has a "group_key". It means a group can be part of another.

### Destination

Here, we only subscribe to the `/dc/group/memory_uptime` topic

```yaml
dc_bridge:
  ros__parameters:
    shipper:
      data_dir: "$HOME/.dc/buffer"
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/group/memory_uptime"]
      time_key: "date"
      time_format: "double"
    vector_forward_host: "127.0.0.1"
    vector_forward_port: 24224
```

## Console output

In the terminal, you can see the result, published every 3 seconds (see the date field), which the is timeframe defined by sync_delay and the maximum polling_interval of the measurements.

Finally, note the new dictionary uses the key defined in the `group_key` measurement_server plugin configuration. They are transferred through the ROS message.

```
[dc_bridge-3] {"date":1788476662.509069,"host":"127.0.0.1","memory":{"flattened":false,"name":"memory","nested":false,"run_id":"169","used":95.77983856201172},"name":"memory_uptime","source_type":"fluent","tag":"dc.group.memory_uptime","tags":[""],"timestamp":"2026-09-03T23:04:22.509068935Z","uptime":{"flattened":false,"name":"uptime","nested":false,"run_id":"169","time":1609047}}
[dc_bridge-3] {"date":1788476665.5074596,"host":"127.0.0.1","memory":{"flattened":false,"name":"memory","nested":false,"run_id":"169","used":95.87796020507812},"name":"memory_uptime","source_type":"fluent","tag":"dc.group.memory_uptime","tags":[""],"timestamp":"2026-09-03T23:04:25.507459531Z","uptime":{"flattened":false,"name":"uptime","nested":false,"run_id":"169","time":1609050}}
[dc_bridge-3] {"date":1788476668.5055394,"host":"127.0.0.1","memory":{"flattened":false,"name":"memory","nested":false,"run_id":"169","used":96.04102325439453},"name":"memory_uptime","source_type":"fluent","tag":"dc.group.memory_uptime","tags":[""],"timestamp":"2026-09-03T23:04:28.505539347Z","uptime":{"flattened":false,"name":"uptime","nested":false,"run_id":"169","time":1609053}}
```
