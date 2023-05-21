# Uptime to stdout

This is the most minimal example to run DC, it collects the system uptime every 5 seconds and sends it to Stdout.

Let's run it:

```bash
$ ros2 launch dc_demos uptime_stdout.launch.py
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
      tags: ["flb_stdout"]
      init_collect: true
    run_id:
      enabled: true
      counter: true
      counter_path: "$HOME/run_id"
      uuid: false
```

**measurement_plugins (Mandatory)**: List all the plugins to enable. This is a custom string that is equal to the measurement plugin dictionary present in the same level. If not listed, will not be loaded.

**uptime.plugin (Mandatory)**: Name of the plugin, if you are not sure which plugin is available, [use the CLI tool](../cli.md) to list them

**uptime.tags (Mandatory)**: This is used by Fluent Bit, to match inputs and outputs. More [here](../concepts.md#tags)

**uptime.polling_interval (Optional)**: Interval to which data is collected in milliseconds

**uptime.enable_validator (Optional)**: Will validate the data against a JSON schema. This file is located in the [dc_measurements package](https://github.com/Minipada/ros2_data_collection/tree/humble/dc_measurements/plugins/measurements/json). You can provide your own using the `json_schema_path` parameter, which we will explore later on

**uptime.debug (Optional)**: More verbose output

**uptime.init_collect (Optional)**: Collect when the node starts instead of waiting for the polling_interval time to pass

**run_id.enabled (Optional)**: Identify which run the robot is. A new one is generated at every start of the node. Uses either a counter that increment at each restart of the node or UUID

**run_id.counter (Optional)**: Enable counter for the run_id

**run_id.counter_path (Optional)**: Path to store the last run. It is expanded with environment variables id

**run_id.uuid (Optional)**: Generate a new run ID by using a random UUID

This will collect the uptime every 5 seconds (including when the node starts), will forward it to the *flb_stdout* destination.

```admonish info

Note that this configuration alone, sent to Fluent Bit will not display the JSON on stdout since it requires the destination_server configuration
```

Find the complete measurements documentation [here](../measurements.md)

### Destination

```yaml
destination_server:
  ros__parameters:
    flb:
      flush: 1
      flb_grace: 1
      log_level: "info"
      storage_path: "/var/log/flb-storage/"
      storage_sync: "full"
      storage_checksum: "off"
      storage_backlog_mem_limit: "1M"
      scheduler_cap: 200
      scheduler_base: 5
      http_server: true
      http_listen: "0.0.0.0"
      http_port: 2020
      in_storage_type: "filesystem"
      in_storage_pause_on_chunks_overlimit: "off"
    destination_plugins: ["flb_stdout"]
    custom_str_params_list: ["robot_name", "id"]
    custom_str_params:
      robot_name:
        name: robot_name
        value: C3PO
      id:
        name: id
        value_from_file: /etc/machine-id
    flb_stdout:
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/measurement/uptime"]
      time_format: "double"
      time_key: "date"
      debug: false
```

#### Destinations

Let's analyze piece by piece. First, we select the plugin to enable, create its section containing the plugin name and the topic to subscribe to. We need the topic list because in the background, the ROS 2 Fluent Bit plugin subscribes to each topic.

**destination_plugins (Mandatory)**: List all the plugins to enable. This is a custom string that is equal to the destination plugin dictionary present in the same level. If not listed, will not be loaded.

**flb_stdout.plugin (Mandatory)**: Plugin to load

**flb_stdout.inputs (Mandatory)**: Topics to which to listen to get the data

**flb_stdout.time_format (Optional)**: Format the data will be printed

**flb_stdout.time_key (Optional)**: Dictionary key from which date will be taken from

**flb_stdout.debug (Optional)**: Verbose output

#### Fluent Bit

Then, we configure Fluent Bit. This is not necessary but it is easy to do by using the ros parameters provided by the node.

**flb.flush (Optional)**: Interval to flush output (seconds)

**flb.flb_grace (Optional)**: Wait time (seconds) on exit

**flb.log_level (Optional)**: Diagnostic level (error/warning/info/debug/trace)

**flb.storage_path (Optional)**: Set an optional location in the file system to store streams and chunks of data. If this parameter is not set, Input plugins can only use in-memory buffering.

**flb.storage_sync (Optional)**: Configure the synchronization mode used to store the data into the file system. It can take the values normal or full.

**flb.storage_checksum (Optional)**: Enable the data integrity check when writing and reading data from the filesystem. The storage layer uses the CRC32 algorithm.

**flb.storage_backlog_mem_limit (Optional)**: If storage.path is set, Fluent Bit will look for data chunks that were not delivered and are still in the storage layer, these are called backlog data. This option configure a hint of maximum value of memory to use when processing these records.

**flb.scheduler_cap (Optional)**: Set a maximum retry time in seconds. The property is supported from v1.8.7.

**flb.scheduler_base (Optional)**: Set a base of exponential backoff. The property is supported from v1.8.7.

**flb.http_server (Optional)**: If true enable statistics HTTP server

**flb.http_listen (Optional)**: Address to listen (e.g. 0.0.0.0)

**flb.http_port (Optional)**: Port to listen (e.g. 8888)

**flb.in_storage_type (Optional)**: Specifies the buffering mechanism to use. It can be memory or filesystem.

**flb.in_storage_pause_on_chunks_overlimit (Optional)**: Specifies if file storage is to be paused when reaching the chunk limit.

#### Inject custom data for each record

Here, we want to append some content in every record: the robot name and its ID. While the robot name comes from a fixed variable in the parameter file, the id comes from the machine-id file.

**custom_str_params_list (Optional)**: Look for those keys in this configuration to add them as keys and values in each record.

**custom_str_params.robot_name (Optional)**: This parameter is loaded since it is mentioned in custom_str_params_list

**custom_str_params.robot_name.name (Optional)**: Key in the dictionary to add

**custom_str_params.robot_name.value (Optional)**: Value associated to the key in the dictionary to add

**custom_str_params.id.name (Optional)**: Key in the dictionary to add

**custom_str_params.id.value_from_file (Optional)**: Value associated to the key in the dictionary to add taken from the content of a file

#### Inject run id at each record

Finally, we set the run id. This is used later on when fetching data for a run. It can come from a counter which is incremented at each start of the node or from a random UUID generated. The counter mechanism writes and read on a file on the system (take care of not deleting it), you can set its path as a parameter.

Find the complete destinations documentation [here](../destinations.md)

## Console output

Now that the node started, let us see what's displayed in the console

```
[component_container_isolated-1] [INFO] [1677668906.621492168] [dc_container]: Load Library: /root/ws/install/dc_measurements/lib/libmeasurement_server_core.so
[component_container_isolated-1] [INFO] [1677668906.634130562] [dc_container]: Found class: rclcpp_components::NodeFactoryTemplate<measurement_server::MeasurementServer>
[component_container_isolated-1] [INFO] [1677668906.634246977] [dc_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<measurement_server::MeasurementServer>
[component_container_isolated-1] [INFO] [1677668906.655803692] [measurement_server]:
	measurement_server lifecycle node launched.
	Waiting on external lifecycle transitions to activate
	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[component_container_isolated-1] [INFO] [1677668906.670233932] [dc_container]: Load Library: /root/ws/install/dc_destinations/lib/libdestination_server_core.so
[component_container_isolated-1] [INFO] [1677668906.675826543] [dc_container]: Found class: rclcpp_components::NodeFactoryTemplate<destination_server::DestinationServer>
[component_container_isolated-1] [INFO] [1677668906.675864319] [dc_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<destination_server::DestinationServer>
[component_container_isolated-1] [INFO] [1677668906.680499515] [destination_server]:
[component_container_isolated-1] 	destination_server lifecycle node launched.
[component_container_isolated-1] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-1] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/destination_server' in container 'dc_container'
[component_container_isolated-1] [INFO] [1677668906.684802172] [dc_container]: Load Library: /opt/ros/humble/lib/libnav2_lifecycle_manager_core.so
[component_container_isolated-1] [INFO] [1677668906.685487672] [dc_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_lifecycle_manager::LifecycleManager>
[component_container_isolated-1] [INFO] [1677668906.685510016] [dc_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_lifecycle_manager::LifecycleManager>
[component_container_isolated-1] [INFO] [1677668906.691075881] [lifecycle_manager_navigation]: Creating
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/lifecycle_manager_navigation' in container 'dc_container'
[component_container_isolated-1] [INFO] [1677668906.692954854] [lifecycle_manager_navigation]: Creating and initializing lifecycle service clients
[component_container_isolated-1] [INFO] [1677668906.695025050] [lifecycle_manager_navigation]: Starting managed nodes bringup...
```

Measurement server and destination server are starting in the Lifecycle, you can read more about it [here](../concepts.md#lifecycle-nodes-and-bond)

Afterward, the measurement_server starts:

```
[component_container_isolated-1] [INFO] [1677668906.695056624] [lifecycle_manager_navigation]: Configuring measurement_server
[component_container_isolated-1] [INFO] [1677668906.695181755] [measurement_server]: Configuring
[component_container_isolated-1] [INFO] [1677668906.698019687] [measurement_server]: Creating measurement plugin uptime: Type dc_measurements/Uptime, Group key: uptime, Polling interval: 5000, Debug: 1, Validator enabled: 1, Schema path: , Tags: [flb_stdout], Init collect: 1, Init Max measurement: 0, Include measurement name: 0, Include measurement plugin name: 0, Remote keys: , Remote prefixes: , Include measurement plugin name: 0, Max measurement on condition: 0, If all condition: , If any condition: , If none condition:
[component_container_isolated-1] [INFO] [1677668906.698932685] [measurement_server]: Configuring uptime
[component_container_isolated-1] [INFO] [1677668906.699937310] [measurement_server]: Done configuring uptime
[component_container_isolated-1] [INFO] [1677668906.700162355] [measurement_server]: Looking for schema at /root/ws/install/dc_measurements/share/dc_measurements/plugins/measurements/json/uptime.json
[component_container_isolated-1] [INFO] [1677668906.700192161] [measurement_server]: schema: {"$schema":"http://json-schema.org/draft-07/schema#","description":"Time the system has been up","properties":{"time":{"description":"Time the system has been up","minimum":0,"type":"integer"}},"title":"Uptime","type":"object"}
```

Plugins are loaded one by one (here only one is, the uptime one) and configured. The configuration for each is displayed and the validation schema is also loaded and its path printed.

Then, the destination_server starts:

```
[component_container_isolated-1] [INFO] [1677668906.700528260] [lifecycle_manager_navigation]: Configuring destination_server
[component_container_isolated-1] [INFO] [1677668906.700620104] [destination_server]: Configuring
[component_container_isolated-1] [INFO] [1677668906.701246047] [destination_server]: Fluent Bit service initialized
[component_container_isolated-1] [INFO] [1677668906.701470896] [destination_server]: Creating destination plugin flb_stdout: Type dc_destinations/FlbStdout, Debug: 0, Time format: double. Time key: date
[component_container_isolated-1] [INFO] [1677668906.702670279] [destination_server]: Configuring Flb plugin flb_stdout
[component_container_isolated-1] [INFO] [1677668906.702871127] [destination_server]: Loaded lua filter. Match=ros2, code=function concatenate(tag, timestamp, record) if (type(record["tags"]) == "table") then record["tags"] = table.concat(record["tags"], ",") end  return 2, timestamp, record end
[component_container_isolated-1] [INFO] [1677668906.702912686] [destination_server]: Loaded rewrite_tag filter. Match=ros2, Rule=$tags .*(flb_stdout).* flb_stdout true
[component_container_isolated-1] [INFO] [1677668906.703021628] [destination_server]: Done configuring Flb plugin flb_stdout
[component_container_isolated-1] [INFO] [1677668906.703049312] [destination_server]: Loading input ros2 shared library /root/ws/install/fluent_bit_plugins/lib/flb-in_ros2.so...
[component_container_isolated-1] [INFO] [1677668906.703818806] [destination_server]: Loaded input ros2 shared library /root/ws/install/fluent_bit_plugins/lib/flb-in_ros2.so
[component_container_isolated-1] [INFO] [1677668906.703899346] [destination_server]: Flb ros2 plugin initialized. ret=0
[component_container_isolated-1] [INFO] [1677668906.703908977] [destination_server]: Starting Flb engine...
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [fluent bit] version=2.0.7, commit=1ab360f79c, pid=31126
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [storage] ver=1.3.0, type=memory+filesystem, sync=full, checksum=off, max_chunks_up=128
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [storage] backlog input plugin: storage_backlog.1
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [cmetrics] version=0.5.7
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [ctraces ] version=0.2.5
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [input:ros2:ros2.0] initializing
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [input:ros2:ros2.0] storage_strategy='filesystem' (memory + filesystem)
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] Started node fluentbit_rclc
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] Created subscriber /dc/measurement/uptime
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [input:storage_backlog:storage_backlog.1] initializing
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [input:storage_backlog:storage_backlog.1] storage_strategy='memory' (memory only)
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [input:storage_backlog:storage_backlog.1] queue memory limit: 976.6K
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [input:emitter:emitter_for_rewrite_tag.2] initializing
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [input:emitter:emitter_for_rewrite_tag.2] storage_strategy='filesystem' (memory + filesystem)
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [output:stdout:stdout.0] worker #0 started
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [http_server] listen iface=0.0.0.0 tcp_port=2020
[component_container_isolated-1] [2023/03/01 11:08:26] [ info] [sp] stream processor started
[component_container_isolated-1] [INFO] [1677668906.744577163] [destination_server]: Started Flb engine
```

It starts the [FlbStdout destination plugin](../destinations/flb_stdout.md), filters used internally by Fluent to edit Timestamps and tags. It then loads the ROS 2 Fluent Bit shared library and initialize it with the topics we provided as parameter.

Afterward, Fluent Bit starts, it prints its version, storage strategy and buffer configuration and lists which plugins are loaded.

Then, the measurement and destination server nodes are activated:

```
[component_container_isolated-1] [INFO] [1677668906.745180268] [lifecycle_manager_navigation]: Activating measurement_server
[component_container_isolated-1] [INFO] [1677668906.745462265] [measurement_server]: Activating
[component_container_isolated-1] [INFO] [1677668906.745558665] [measurement_server]: Activating uptime
[component_container_isolated-1] [INFO] [1677668906.746014639] [measurement_server]: Creating bond (measurement_server) to lifecycle manager.
[component_container_isolated-1] [INFO] [1677668906.853199899] [lifecycle_manager_navigation]: Server measurement_server connected with bond.
[component_container_isolated-1] [INFO] [1677668906.853316226] [lifecycle_manager_navigation]: Activating destination_server
[component_container_isolated-1] [INFO] [1677668906.853620571] [destination_server]: Activating
[component_container_isolated-1] [INFO] [1677668906.853722659] [destination_server]: Activating flb_stdout
[component_container_isolated-1] [INFO] [1677668906.853796078] [destination_server]: Creating bond (destination_server) to lifecycle manager.
[component_container_isolated-1] [INFO] [1677668906.961666207] [lifecycle_manager_navigation]: Server destination_server connected with bond.
[component_container_isolated-1] [INFO] [1677668906.961777773] [lifecycle_manager_navigation]: Managed nodes are active
[component_container_isolated-1] [INFO] [1677668906.961820078] [lifecycle_manager_navigation]: Creating bond timer...
```

Finally, we see the data:
```
[component_container_isolated-1] [{"date":1677668906.745817,"time":92395,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}]
[component_container_isolated-1] [{"date":1677668911.700309,"time":92400,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}]
[component_container_isolated-1] [{"date":1677668916.70031,"time":92405,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}]
[component_container_isolated-1] [{"date":1677668921.700388,"time":92410,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}]
[component_container_isolated-1] [{"date":1677668926.700422,"time":92415,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"218"}]
```

So...what happened?

1. The measurement plugin starts publishing data to /dc/measurement/uptime, which contains the JSON, tags and timestamp of the message
2. Run ID and robot_name is appended in the JSON
3. The ROS 2 Fluent Bit plugin, which subscribes to this topic, receive the data and pass it on to Fluent Bit
4. Fluent Bit receives it, applies the timestamp filter (which modifies the timestamp to the desired format)
5. Fluent Bit applies changes the tag with another filter. This tag is used to match to the destination afterward.
6. Data is flushed
7. The Fluent Bit stdout output plugin receives the data because tags match (the flb_stdout tag is used) and forwards it to Stdout
