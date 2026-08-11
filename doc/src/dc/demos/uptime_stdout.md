# Uptime to stdout

This is the most minimal example to run DC, it collects the system uptime every 5 seconds and sends it to Stdout.

Let's run it:

```bash
ros2 launch dc_demos uptime_stdout.launch.py
```

At the end, the data is displayed:
```
[component_container_isolated-1] [{"date":1677668906.745817,"time":92395,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}]
[component_container_isolated-1] [{"date":1677668911.700309,"time":92400,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}]
[component_container_isolated-1] [{"date":1677668916.70031,"time":92405,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}]
[component_container_isolated-1] [{"date":1677668921.700388,"time":92410,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}]
[component_container_isolated-1] [{"date":1677668926.700422,"time":92415,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}]
```

This launchfile is a wrapper of [dc_bringup/launch/bringup.launch.py](https://github.com/Minipada/ros2_data_collection/blob/humble/dc_bringup/launch/dc_bringup.launch.py) which loads a [custom yaml configuration](https://github.com/Minipada/ros2_data_collection/blob/humble/dc_demos/params/uptime_stdout.yaml)

## Configuration
### Measurement

```yaml
measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
      polling_interval: 5000
      enable_validator: true
      debug: true
      init_collect: true
    custom_key_str_list: ["robot_name", "id"]
    custom_keys_str:
      robot_name:
        name: robot_name
        value: C3PO
      id:
        name: id
        value_from_file: /etc/machine-id
    run_id:
      enabled: true
      counter: true
      counter_path: "$HOME/run_id"
      uuid: false
```

**measurement_plugins (Mandatory)**: List all the plugins to enable. This is a custom string that is equal to the measurement plugin dictionary present in the same level. If not listed, will not be loaded.

**uptime.plugin (Mandatory)**: Name of the plugin, if you are not sure which plugin is available, [use the CLI tool](../cli.md) to list them

**uptime.polling_interval (Optional)**: Interval to which data is collected in milliseconds

**uptime.enable_validator (Optional)**: Will validate the data against a JSON schema. This file is located in the [dc_measurements package](https://github.com/Minipada/ros2_data_collection/tree/humble/dc_measurements/plugins/measurements/json). You can provide your own using the `json_schema_path` parameter, which we will explore later on

**uptime.debug (Optional)**: More verbose output

**uptime.init_collect (Optional)**: Collect when the node starts instead of waiting for the polling_interval time to pass

**run_id.enabled (Optional)**: Identify which run the robot is. A new one is generated at every start of the node. Uses either a counter that increment at each restart of the node or UUID

**run_id.counter (Optional)**: Enable counter for the run_id

**run_id.counter_path (Optional)**: Path to store the last run. It is expanded with environment variables id

**run_id.uuid (Optional)**: Generate a new run ID by using a random UUID

This will collect the uptime every 5 seconds (including when the node starts), will forward it to the *console* destination.

#### Inject custom data for each record

Here, we want to append some content in every record: the robot name and its ID. While the robot name comes from a fixed variable in the parameter file, the id comes from the machine-id file.

**custom_key_str_list (Optional)**: Look for those keys in this configuration to add them as keys and values in each record.

**custom_keys_str.robot_name (Optional)**: This parameter is loaded since it is mentioned in custom_key_str_list

**custom_keys_str.robot_name.name (Optional)**: Key in the dictionary to add

**custom_keys_str.robot_name.value (Optional)**: Value associated to the key in the dictionary to add

**custom_keys_str.id.name (Optional)**: Key in the dictionary to add

**custom_keys_str.id.value_from_file (Optional)**: Value associated to the key in the dictionary to add taken from the content of a file


```admonish info

Note that this configuration alone will not display the JSON on stdout since it requires the dc_bridge configuration below
```

Find the complete measurements documentation [here](../measurements.md)

### Destination

```yaml
dc_bridge:
  ros__parameters:
    shipper:
      data_dir: "$HOME/.dc/buffer"
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/uptime"]
      time_key: "date"
      time_format: "double"
    vector_forward_host: "127.0.0.1"
    vector_forward_port: 24224
```

#### Destinations

Let's analyze piece by piece. `dc_bridge` is the single C++ node that owns every Destination; each entry in the `destinations` list names a section, defined below it, that describes where data goes. We need the topic list on each Destination because the Bridge subscribes to those topics itself and forwards what it receives to an external [Vector](https://vector.dev) process over the shipper ingest protocol.

**destinations (Mandatory)**: List all the Destinations to enable. Each name must have a matching section at the same level.

**console.type (Mandatory)**: One of the four blessed Destination types (`postgres`, `s3`, `file`, `console`). `console` prints JSON to `dc_bridge`'s own stdout — handy for demos and debugging.

**console.receives (Optional)**: `records` (default) or `files`.

**console.inputs (Mandatory)**: Topics to which to listen to get the data.

**console.time_format (Optional)**: Format the data's timestamp will be printed as (`double` or `iso8601`).

**console.time_key (Optional)**: Dictionary key the timestamp is written under.

`dc_bridge` itself needs no engine tuning of the kind the old embedded Fluent Bit shipper required (buffering, scheduler backoff, HTTP stats server, …) — Vector, the external shipper process it forwards to, owns its own on-disk buffering and is configured from the `dc_bridge`/`destinations` block above; see [ADR-0002](https://github.com/Minipada/ros2_data_collection/blob/jazzy/docs/adr/0002-vector-as-default-shipper.md) for why that split exists.

#### Inject run id at each record

Finally, we set the run id. This is used later on when fetching data for a run. It can come from a counter which is incremented at each start of the node or from a random UUID generated. The counter mechanism writes and read on a file on the system (take care of not deleting it), you can set its path as a parameter.

Find the complete destinations documentation [here](../destinations.md)

## Console output

Now that the node started, let us see what's displayed in the console.

Measurement server and `dc_bridge` are started in the Lifecycle, you can read more about it [here](../concepts.md#lifecycle-nodes-and-bond). Per [ADR-0006](https://github.com/Minipada/ros2_data_collection/blob/jazzy/docs/adr/0006-bridge-outside-lifecycle-manager.md), the lifecycle manager waits on a `bridge_ready_gate` before activating the collection nodes:

```
[bridge_ready_gate-1] dc_bridge reports ready; activating collection nodes.
[dc_bridge-1] [INFO] [dc_bridge]: Rendered Vector config to /tmp/dc_bridge/vector.toml, destinations: [console]
[dc_bridge-1] [INFO] [dc_bridge]: Bridge ready
```

`dc_bridge` renders the `destinations` block above into a Vector config, launches (or reloads) the external Vector process pointed at it, and only then reports ready — see [ADR-0002](https://github.com/Minipada/ros2_data_collection/blob/jazzy/docs/adr/0002-vector-as-default-shipper.md) for why Vector runs as its own process rather than embedded in the Bridge.

Finally, we see the data, now printed by Vector's own `console` sink rather than by the Bridge itself:
```
{"date":1677668906.745817,"time":92395,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}
{"date":1677668911.700309,"time":92400,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}
{"date":1677668916.70031,"time":92405,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}
{"date":1677668921.700388,"time":92410,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}
{"date":1677668926.700422,"time":92415,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}
```

So...what happened?

1. The measurement plugin starts publishing data to /dc/measurement/uptime, which contains the JSON and timestamp of the message
2. Run ID and robot_name is appended in the JSON
3. `dc_bridge`, which subscribes to this topic directly, receives the data and forwards it to Vector over the shipper ingest protocol
4. Vector's generated config applies a `remap` transform that writes the configured `time_key` in the requested `time_format`
5. Vector's `console` sink, the only one matching the `console` Destination we configured, prints the JSON to stdout
