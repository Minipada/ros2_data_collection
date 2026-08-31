# Configuration examples

Through minimal code examples, you will learn how to collect and send data with DC.

It will progressively present all features.

```admonish info
By here, you must have built the workspace following the [setup guide](./setup.md).
```

Copy the configuration and save it as a yaml file, and then run:

```bash
ros2 launch dc_bringup dc_bringup.launch.py params_file:="my_file.yaml"
```

Examples that use a Group also need the Group node:

```bash
ros2 launch dc_bringup dc_bringup.launch.py params_file:="my_file.yaml" group_node:=True
```

```admonish tip
Every example follows the same shape: a `dc_bridge` block declaring **Destinations** and
the topics each one `inputs`, and a `measurement_server` block declaring **Measurements**
and the topic each one publishes on. Routing is the overlap between the two lists —
nothing on the Measurement side names a Destination. If you are migrating a DC 1.x
configuration, start with the [migration guide](./migration.md).
```

## Running the examples
### Example 1: Uptime to the console every second

```yaml
dc_bridge:                                    # Bridge (Shipper) node configuration
  ros__parameters:
    destinations: ["console"]                 # List of Destination names to enable
    console:                                  # Destination name, you choose
      type: console                           # Blessed Destination type, fixed
      receives: records
      inputs: ["/dc/measurement/uptime"]      # Same as topic_output in the uptime measurement in measurement_server

measurement_server:                           # Measurement node configuration
  ros__parameters:
    measurement_plugins: ["uptime"]           # List of measurement plugins names to enable
    uptime:                                   # Plugin name, you choose
      plugin: "dc_measurements/Uptime"        # Plugin class name, fixed
      topic_output: "/dc/measurement/uptime"  # Topic where data will be published
```

### Example 2: Uptime to the console with ISO 8601 timestamps

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/uptime"]
      time_key: "date"                       # Field the normalized timestamp is written to
      time_format: "iso8601"                 # "epoch_nanos" (default) | "iso8601" | "double"

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
```

### Example 3: Uptime to the console only at start and 3 times

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/uptime"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
      init_max_measurements: 3               # Maximum records to collect
```

### Example 4: CPU and Memory to the console every 5 seconds forever

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/cpu", "/dc/measurement/memory"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["memory", "cpu"]
    memory:
      plugin: "dc_measurements/Memory"
      topic_output: "/dc/measurement/memory"
      polling_interval: 5000                  # Interval to which data is collected in milliseconds
    cpu:
      plugin: "dc_measurements/Cpu"
      topic_output: "/dc/measurement/cpu"
      polling_interval: 5000                  # Interval to which data is collected in milliseconds
```

### Example 5: CPU and Memory as a group to the console every 5 seconds forever

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/group/cpu_memory"]        # Group to create

group_server:                                 # Group server configuration
  ros__parameters:
    groups: ["cpu_memory"]
    cpu_memory:
      inputs: ["/dc/measurement/cpu", "/dc/measurement/memory"] # Topics which are subscribed
      output: "/dc/group/cpu_memory"          # Topic where result will be published
      sync_delay: 5.0                         # How long to queue up messages before passing them through.
      group_key: "cpu_memory"

measurement_server:
  ros__parameters:
    measurement_plugins: ["memory", "cpu"]
    memory:
      plugin: "dc_measurements/Memory"
      topic_output: "/dc/measurement/memory"
      polling_interval: 5000
    cpu:
      plugin: "dc_measurements/Cpu"
      topic_output: "/dc/measurement/cpu"
      polling_interval: 5000
```

### Example 6: Custom ROS message to the console every 2 seconds forever

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/my_string_stamped"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["my_string_stamped"]
    my_string_stamped:
      plugin: "dc_measurements/StringStamped"           # Plugin that allow to publish from your nodes
      topic_output: "/dc/measurement/my_string_stamped" # Topic where the Record is republished
      topic: "/hello-world"                             # Input topic where you are publishing
      polling_interval: 2000
      enable_validator: false                           # By default, StringStamped message does not have a JSON schema since it uses custom input data
```

You will then need in another terminal to publish data on the input topic (`/hello-world`)

```bash
ros2 topic pub -r 1 /hello-world dc_interfaces/msg/StringStamped  "{data: '{\"hello\":\"world\"}'}"
```

### Example 7: Custom ROS message to the console every time it is published

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/my_string_stamped"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["my_string_stamped"]
    my_string_stamped:
      plugin: "dc_measurements/StringStamped"
      topic_output: "/dc/measurement/my_string_stamped"
      topic: "/hello-world"
      enable_validator: false
      timer_based: false                                 # Get all data published on the input topic. Ignores polling_interval
```

### Example 8: Uptime to PostgreSQL, and to the console at the same time

A Record is delivered to every Destination that lists its topic — listing the same topic
twice is how you fan out.

```yaml
dc_bridge:
  ros__parameters:
    shipper:
      data_dir: "$HOME/.dc/buffer"            # Where the Shipper keeps its disk buffer
    destinations: ["pgsql", "console"]
    pgsql:
      type: postgres
      receives: records
      inputs: ["/dc/measurement/uptime"]
      host: "127.0.0.1"
      port: 5432
      user: "dc"
      password: "$DC_PG_PASSWORD"             # $VAR / ${VAR} are read from the environment
      database: "dc"
      table: "dc"
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/uptime"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
```

```admonish warning
The Shipper's `postgres` Destination maps a Record's top-level JSON keys onto **existing**
columns; it does not create tables or columns. Create the table before starting DC.
```

### Example 9: Camera images to object storage, with their metadata in PostgreSQL

Files (images, maps, videos) never travel through the Shipper. A `receives: files`
Destination is served by the Bridge's Uploader, and the per-File status Records it
produces go to whichever Destination `files.metadata_destination` names. The Uploader's
durable upload intent queue and multipart-resume state live under `uploader.data_dir`,
separate from the Shipper's own disk buffer under `shipper.data_dir` — set both, as below,
so it's obvious on disk (and later in volume mounts) which files belong to which owner.
If `uploader.data_dir` is omitted it defaults to `shipper.data_dir`, so existing configs
that only set the latter keep working unchanged.

```yaml
dc_bridge:
  ros__parameters:
    shipper:
      data_dir: "$HOME/.dc/shipper"
    uploader:
      data_dir: "$HOME/.dc/uploader"
    destinations: ["pgsql", "rustfs"]
    pgsql:                                    # the Records, and the File status log
      type: postgres
      receives: records
      inputs: ["/dc/measurement/camera"]
      host: "127.0.0.1"
      port: 5432
      user: "dc"
      password: "$DC_PG_PASSWORD"
      database: "dc"
      table: "dc"
    rustfs:                                   # the File bytes
      type: s3
      receives: files
      inputs: ["/dc/measurement/camera"]
      bucket: "dc-files"
      endpoint: "http://127.0.0.1:9000"       # omit for AWS S3
      region: "us-east-1"
      access_key_id: "rustfsadmin"
      secret_access_key: "$DC_S3_SECRET"
      force_path_style: true                  # path-style addressing for self-hosted stores
    files:
      delete_when_sent: true                  # delete locally once verified remotely
      metadata_destination: "pgsql"

measurement_server:
  ros__parameters:
    measurement_plugins: ["camera"]
    camera:
      plugin: "dc_measurements/Camera"
      topic_output: "/dc/measurement/camera"
      cam_topic: "/camera/image_raw"
      cam_name: "camera"
      save_detections_img: true
      save_inspected_path: "camera/inspected/%Y-%m-%dT%H-%M-%S"
      detection_modules: ["barcode"]
      remote_keys: ["rustfs"]                 # must equal the receives: files Destination name
      remote_prefixes: [""]
```

### Example 10: A Destination DC does not bless, via the passthrough

Any sink in [Vector's catalog](https://vector.dev/docs/reference/configuration/sinks/) is
reachable by handing raw Shipper configuration through, consuming the public `dc.<tag>`
route for the topic you want.

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
    custom_config_files: ["$HOME/.dc/http_sink.toml"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
```

```toml
# $HOME/.dc/http_sink.toml — raw Vector configuration, merged as-is
[sinks.my_api]
type = "http"
inputs = ["dc.dc.measurement.uptime"]   # /dc/measurement/uptime's public route
uri = "http://127.0.0.1:8080/ingest"
encoding.codec = "json"
```

Now that you know how it works, you can set up your own Measurements and Destinations —
see [Measurements](./measurements.md) and [Destinations](./destinations.md) for every
parameter.
