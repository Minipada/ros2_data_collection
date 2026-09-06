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
nothing on the Measurement side names a Destination.
```

```admonish info title="Why `file` + a passthrough snippet, not a blessed `console` Destination"
Every "to the console" example below prints through a passthrough `console` sink loaded
via `custom_config_files`, not a blessed `console` Destination — per
[ADR-0003](./adr/0003-blessed-destinations-plus-passthrough.md), `console` (along with
`postgres` and `s3`) moved from the blessed ROS-param form to a passthrough recipe (#471),
being a pure Vector-sink wrapper with no DC-specific logic. `destinations` still names a
`file` Destination in each example: `dc_bridge` derives its ROS subscriptions and
`dc.<tag>` routes from `destinations` alone, never from a passthrough snippet's `inputs`,
so a cheap `file` anchor is what actually creates the route the snippet consumes. See
[Destinations: Recipes](./destinations.md#recipes-postgres-s3-console-via-passthrough) for
the recipe this reuses throughout, and [Passthrough](./destinations.md#passthrough-custom_config_files)
for the underlying mechanism.

Save this once as `~/.dc/console_sink.toml`, and update its `inputs` to match whichever
example you're running (each example below says what to set it to):

    [sinks.debug_console]
    type = "console"
    inputs = ["dc.dc.measurement.uptime"]   # <- change this to match the example
    target = "stdout"

    [sinks.debug_console.encoding]
    codec = "json"
```

## Running the examples
### Example 1: Uptime to the console every second

`console_sink.toml`'s `inputs`: `["dc.dc.measurement.uptime"]` (the default above).

```yaml
dc_bridge:                                    # Bridge (Shipper) node configuration
  ros__parameters:
    destinations: ["records_log"]             # List of Destination names to enable
    records_log:                              # Destination name, you choose
      type: file                              # Blessed Destination type -- the passthrough's anchor
      receives: records
      inputs: ["/dc/measurement/uptime"]      # Same as topic_output in the uptime measurement in measurement_server
      path: "/tmp/dc/example1_records.ndjson"
    custom_config_files: ["$HOME/.dc/console_sink.toml"]

measurement_server:                           # Measurement node configuration
  ros__parameters:
    measurement_plugins: ["uptime"]           # List of measurement plugins names to enable
    uptime:                                   # Plugin name, you choose
      plugin: "dc_measurements/Uptime"        # Plugin class name, fixed
      topic_output: "/dc/measurement/uptime"  # Topic where data will be published
```

### Example 2: Uptime to the console with ISO 8601 timestamps

`console_sink.toml`'s `inputs`: `["dc.dc.measurement.uptime"]` (unchanged from Example 1).

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["records_log"]
    records_log:
      type: file
      receives: records
      inputs: ["/dc/measurement/uptime"]
      path: "/tmp/dc/example2_records.ndjson"
      time_key: "date"                       # Field the normalized timestamp is written to
      time_format: "iso8601"                 # "epoch_nanos" (default) | "iso8601" | "double"
    custom_config_files: ["$HOME/.dc/console_sink.toml"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
```

### Example 3: Uptime to the console only at start and 3 times

`console_sink.toml`'s `inputs`: `["dc.dc.measurement.uptime"]` (unchanged from Example 1).

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["records_log"]
    records_log:
      type: file
      receives: records
      inputs: ["/dc/measurement/uptime"]
      path: "/tmp/dc/example3_records.ndjson"
    custom_config_files: ["$HOME/.dc/console_sink.toml"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
      init_max_measurements: 3               # Maximum records to collect
```

### Example 4: CPU and Memory to the console every 5 seconds forever

`console_sink.toml`'s `inputs`: `["dc.dc.measurement.cpu", "dc.dc.measurement.memory"]`.

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["records_log"]
    records_log:
      type: file
      receives: records
      inputs: ["/dc/measurement/cpu", "/dc/measurement/memory"]
      path: "/tmp/dc/example4_records.ndjson"
    custom_config_files: ["$HOME/.dc/console_sink.toml"]

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

`console_sink.toml`'s `inputs`: `["dc.dc.group.cpu_memory"]`.

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["records_log"]
    records_log:
      type: file
      receives: records
      inputs: ["/dc/group/cpu_memory"]        # Group to create
      path: "/tmp/dc/example5_records.ndjson"
    custom_config_files: ["$HOME/.dc/console_sink.toml"]

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

`console_sink.toml`'s `inputs`: `["dc.dc.measurement.my_string_stamped"]`.

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["records_log"]
    records_log:
      type: file
      receives: records
      inputs: ["/dc/measurement/my_string_stamped"]
      path: "/tmp/dc/example6_records.ndjson"
    custom_config_files: ["$HOME/.dc/console_sink.toml"]

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

`console_sink.toml`'s `inputs`: `["dc.dc.measurement.my_string_stamped"]` (unchanged from Example 6).

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["records_log"]
    records_log:
      type: file
      receives: records
      inputs: ["/dc/measurement/my_string_stamped"]
      path: "/tmp/dc/example7_records.ndjson"
    custom_config_files: ["$HOME/.dc/console_sink.toml"]

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

A Record is delivered to every Vector sink that consumes its route — listing the same
`dc.<tag>` route in two sinks' `inputs` is how you fan out. Both PostgreSQL and console are
reached through the passthrough here (per [ADR-0003](./adr/0003-blessed-destinations-plus-passthrough.md),
neither is a blessed Destination any more — see
[Destinations: Recipes](./destinations.md#recipes-postgres-s3-console-via-passthrough)); a
single `file` anchor creates the one route both passthrough sinks consume.

```yaml
dc_bridge:
  ros__parameters:
    shipper:
      data_dir: "$HOME/.dc/buffer"            # Where the Shipper keeps its disk buffer
    destinations: ["records_log"]
    records_log:
      type: file
      receives: records
      inputs: ["/dc/measurement/uptime"]
      path: "/tmp/dc/example8_records.ndjson"
    custom_config_files: ["$HOME/.dc/example8_sink.toml"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
```

```toml
# ~/.dc/example8_sink.toml
[sinks.pgsql]
type = "postgres"
inputs = ["dc.dc.measurement.uptime"]
endpoint = "postgres://dc:$DC_PG_PASSWORD@127.0.0.1:5432/dc"  # user:password@host:port/database
table = "dc"

[sinks.pgsql.buffer]
type = "disk"
max_size = 268435488

[sinks.debug_console]
type = "console"
inputs = ["dc.dc.measurement.uptime"]
target = "stdout"

[sinks.debug_console.encoding]
codec = "json"
```

```admonish warning
Vector's `postgres` sink maps a Record's top-level JSON keys onto **existing**
columns; it does not create tables or columns. Create the table before starting DC.
Unlike the blessed form's `password`, the `$DC_PG_PASSWORD` above is Vector's *own*
`${VAR}` interpolation, off by default in the vendored Vector binary `dc_bridge` spawns
— see the warning in [Destinations: Recipes](./destinations.md#recipes-postgres-s3-console-via-passthrough).
```

### Example 9: Camera images to object storage, with their metadata in PostgreSQL

Files (images, maps, videos) never travel through the Shipper. A `receives: files`
Destination is served by `dc_uploader`, a separate process with its own Shipper
connection ([ADR-0014](./adr/0014-uploader-runs-as-its-own-process.md)), and the
per-File status Records it produces go to whichever Destination
`files.metadata_destination` names. `dc_uploader`'s durable upload intent queue and
multipart-resume state live under `uploader.data_dir`,
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
    destinations: ["records_log", "rustfs"]
    records_log:                              # anchor for the Records, and for the File status log
      type: file
      receives: records
      inputs: ["/dc/measurement/camera"]
      path: "/tmp/dc/example9_records.ndjson"
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
      metadata_destination: "records_log"     # must name a `receives: records` Destination -- a passthrough sink id isn't eligible
    custom_config_files: ["$HOME/.dc/example9_sink.toml"]

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

```toml
# ~/.dc/example9_sink.toml -- consumes both routes records_log creates: the camera
# measurement's own topic, and the dc.files Tag it gains from being named as
# files.metadata_destination.
[sinks.pgsql]
type = "postgres"
inputs = ["dc.dc.measurement.camera", "dc.dc.files"]
endpoint = "postgres://dc:$DC_PG_PASSWORD@127.0.0.1:5432/dc"
table = "dc"

[sinks.pgsql.buffer]
type = "disk"
max_size = 268435488
```

`rustfs` stays a blessed Destination: `receives: files` is served entirely by
`dc_uploader` reading these same ROS params, never by a Vector sink, so there is no
passthrough equivalent for it to migrate to (see
[Destinations: Recipes](./destinations.md#recipes-postgres-s3-console-via-passthrough)).

### Example 10: A Destination DC does not bless, via the passthrough

Any sink in [Vector's catalog](https://vector.dev/docs/reference/configuration/sinks/) is
reachable by handing raw Shipper configuration through, consuming the public `dc.<tag>`
route for the topic you want.

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
      path: "/tmp/dc/example10_records.ndjson"
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
