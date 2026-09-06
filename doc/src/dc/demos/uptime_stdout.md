# Uptime to stdout

This is the most minimal example to run DC, it collects the system uptime every 5 seconds and sends it to Stdout.

Copy the passthrough sink into place, then run it:

```bash
mkdir -p ~/.dc && cp "$(ros2 pkg prefix dc_demos)/share/dc_demos/config/uptime_stdout_sink.toml" ~/.dc/
ros2 launch dc_demos uptime_stdout.launch.py
```

At the end, the data is displayed. Every Destination — the passthrough `console` sink
below included — goes through the external Vector Shipper
([ADR-0002](../adr/0002-vector-as-default-shipper.md), [Destinations](../destinations.md)),
so this is Vector's own event object after ingesting the Record over the Fluent-forward
protocol: `source_type`, `tag`, `host` and `timestamp` are Vector's, not the Bridge's:
```
[dc_bridge-2] {"custom_keys":["robot_name","time"],"date":1788476609.152106,"flattened":false,"host":"127.0.0.1","name":"uptime","nested":false,"robot_name":"C3PO","run_id":"169","source_type":"fluent","tag":"dc.measurement.uptime","time":1608993,"timestamp":"2026-09-03T23:03:29.152105868Z"}
[dc_bridge-2] {"custom_keys":["robot_name","time"],"date":1788476614.1494331,"flattened":false,"host":"127.0.0.1","name":"uptime","nested":false,"robot_name":"C3PO","run_id":"169","source_type":"fluent","tag":"dc.measurement.uptime","time":1608998,"timestamp":"2026-09-03T23:03:34.149433151Z"}
[dc_bridge-2] {"custom_keys":["robot_name","time"],"date":1788476619.149286,"flattened":false,"host":"127.0.0.1","name":"uptime","nested":false,"robot_name":"C3PO","run_id":"169","source_type":"fluent","tag":"dc.measurement.uptime","time":1609003,"timestamp":"2026-09-03T23:03:39.149286154Z"}
```

This launchfile is a wrapper of [dc_bringup/launch/dc_bringup.launch.py](https://github.com/Minipada/ros2_data_collection/blob/jazzy/dc_bringup/launch/dc_bringup.launch.py) which loads a [custom yaml configuration](https://github.com/Minipada/ros2_data_collection/blob/jazzy/dc_demos/params/uptime_stdout.yaml)

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

**uptime.enable_validator (Optional)**: Will validate the data against a JSON schema. This file is located in the [dc_measurements package](https://github.com/Minipada/ros2_data_collection/tree/jazzy/dc_measurements/plugins/measurements/json). You can provide your own using the `json_schema_path` parameter, which we will explore later on

**uptime.debug (Optional)**: More verbose output

**uptime.init_collect (Optional)**: Collect when the node starts instead of waiting for the polling_interval time to pass

**run_id.enabled (Optional)**: Identify which run the robot is. A new one is generated at every start of the node. Uses either a counter that increment at each restart of the node or UUID

**run_id.counter (Optional)**: Enable counter for the run_id

**run_id.counter_path (Optional)**: Path to store the last run. It is expanded with environment variables id

**run_id.uuid (Optional)**: Generate a new run ID by using a random UUID

This will collect the uptime every 5 seconds (including when the node starts), will forward it to the *console* passthrough sink below.

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
    destinations: ["records_log"]
    records_log:
      type: file
      receives: records
      inputs: ["/dc/measurement/uptime"]
      path: "/tmp/dc/uptime_stdout_records.ndjson"
      time_key: "date"
      time_format: "double"
    custom_config_files: ["$HOME/.dc/uptime_stdout_sink.toml"]
    vector_forward_host: "127.0.0.1"
    vector_forward_port: 24224
```

```toml
# ~/.dc/uptime_stdout_sink.toml
[sinks.debug_console]
type = "console"
inputs = ["dc.dc.measurement.uptime"]
target = "stdout"

[sinks.debug_console.encoding]
codec = "json"
```

#### Destinations

Let's analyze piece by piece. `dc_bridge` is the single C++ node that owns every Destination; each entry in the `destinations` list names a section, defined below it, that describes where data goes. We need the topic list on each Destination because the Bridge subscribes to those topics itself and forwards what it receives to an external [Vector](https://vector.dev) process over the shipper ingest protocol.

**destinations (Mandatory)**: List all the Destinations to enable. Each name must have a matching section at the same level.

**records_log.type (Mandatory)**: One of the blessed Destination types (`postgres`, `s3`, `file`, `console`, `vector`). `file` writes each Record as a JSON line to `path`, and is the cheapest anchor to give `destinations` when the actual output you want comes from a `custom_config_files` passthrough sink below — dc_bridge derives its ROS subscriptions and `dc.<tag>` routes from `destinations` alone, never from a passthrough snippet's `inputs`. (Printing straight to stdout used a blessed `console` Destination in earlier versions of this demo; per [ADR-0003](../adr/0003-blessed-destinations-plus-passthrough.md), `console` — along with `postgres` and `s3` — has moved to the passthrough recipe below, since it's a pure Vector-sink wrapper with no DC-specific logic. See [Destinations: Recipes](../destinations.md#recipes-postgres-s3-console-via-passthrough).)

**records_log.receives (Optional)**: `records` (default) or `files`.

**records_log.inputs (Mandatory)**: Topics to which to listen to get the data.

**records_log.path (Mandatory for `file`)**: Absolute path Vector writes each JSON line to (Vector, not the Bridge, expands this — no `$HOME`).

**records_log.time_format (Optional)**: Format the data's timestamp will be printed as (`epoch_nanos` (default), `iso8601` or `double`).

**records_log.time_key (Optional)**: Dictionary key the timestamp is written under.

**custom_config_files (Optional)**: Raw Vector config snippets, merged as-is alongside what `dc_bridge` itself renders. The `uptime_stdout_sink.toml` above defines a plain Vector `console` sink consuming the public `dc.dc.measurement.uptime` route that `records_log`'s `inputs` created — this is what actually prints to stdout; `records_log` itself just writes the same Records to disk as the passthrough's required anchor. See [Destinations: Passthrough](../destinations.md#passthrough-custom_config_files).

`dc_bridge` itself needs no engine tuning of the kind the old embedded Fluent Bit shipper required (buffering, scheduler backoff, HTTP stats server, …) — Vector, the external shipper process it forwards to, owns its own on-disk buffering and is configured from the `dc_bridge`/`destinations` block above; see [ADR-0002](../adr/0002-vector-as-default-shipper.md) for why that split exists.

#### Inject run id at each record

Finally, we set the run id. This is used later on when fetching data for a run. It can come from a counter which is incremented at each start of the node or from a random UUID generated. The counter mechanism writes and read on a file on the system (take care of not deleting it), you can set its path as a parameter.

Find the complete destinations documentation [here](../destinations.md)

## Console output

Now that the node started, let us see what's displayed in the console.

Measurement server and `dc_bridge` are started in the Lifecycle, you can read more about it [here](../concepts.md#lifecycle-nodes-and-bond). Per [ADR-0006](../adr/0006-bridge-outside-lifecycle-manager.md), the lifecycle manager waits on a `bridge_ready_gate` before activating the collection nodes:

```
[INFO] [bridge_ready_gate-3]: process started with pid [31]
[dc_bridge-2] [INFO] [1788478636.751773322] [dc_bridge]: dc_bridge up: 1 subscribed topic(s), supervising /root/ws/install/vector_vendor/lib/vector_vendor/vector
[bridge_ready_gate-3] [INFO] [1788478637.212874385] [bridge_ready_gate]: Bridge is ready: vector is accepting connections
[INFO] [bridge_ready_gate-3]: process has finished cleanly [pid 31]
[INFO] [launch.user]: dc_bridge reports ready; activating collection nodes.
```

`dc_bridge` renders the `destinations` block above into a Vector config and launches (or reloads) the external Vector process pointed at it; `bridge_ready_gate` only lets the launch continue once Vector is actually accepting connections — see [ADR-0002](../adr/0002-vector-as-default-shipper.md) for why Vector runs as its own process rather than embedded in the Bridge.

Finally, we see the data, now printed by the passthrough `console` sink rather than by the Bridge itself:
```
[dc_bridge-2] {"custom_keys":["robot_name","time"],"date":1788476609.152106,"flattened":false,"host":"127.0.0.1","name":"uptime","nested":false,"robot_name":"C3PO","run_id":"169","source_type":"fluent","tag":"dc.measurement.uptime","time":1608993,"timestamp":"2026-09-03T23:03:29.152105868Z"}
[dc_bridge-2] {"custom_keys":["robot_name","time"],"date":1788476614.1494331,"flattened":false,"host":"127.0.0.1","name":"uptime","nested":false,"robot_name":"C3PO","run_id":"169","source_type":"fluent","tag":"dc.measurement.uptime","time":1608998,"timestamp":"2026-09-03T23:03:34.149433151Z"}
[dc_bridge-2] {"custom_keys":["robot_name","time"],"date":1788476619.149286,"flattened":false,"host":"127.0.0.1","name":"uptime","nested":false,"robot_name":"C3PO","run_id":"169","source_type":"fluent","tag":"dc.measurement.uptime","time":1609003,"timestamp":"2026-09-03T23:03:39.149286154Z"}
```

So...what happened?

1. The measurement plugin starts publishing data to /dc/measurement/uptime, which contains the JSON and timestamp of the message
2. Run ID and robot_name is appended in the JSON
3. `dc_bridge`, which subscribes to this topic directly, receives the data and forwards it to Vector over the shipper ingest protocol
4. Vector's generated config applies a `remap` transform that writes the configured `time_key` in the requested `time_format`, and routes the Record onto its public `dc.dc.measurement.uptime` route
5. The passthrough `console` sink from `uptime_stdout_sink.toml`, consuming that route, prints the JSON to stdout — `records_log`'s own `file` sink writes the same Record to disk in parallel
