# Migrating from DC 1.x to DC 2.0

DC 1.x (the `humble` line) embedded Fluent Bit inside the ROS 2 process and exposed one
pluginlib **destination plugin** per output (`flb_pgsql`, `flb_minio`, `flb_s3`, …).
DC 2.0 (the `jazzy` line) retires that layer: an external **Shipper** process
([Vector](https://vector.dev/)) does the buffering and delivery, and the **Bridge**
(`dc_bridge`) renders the Shipper's whole configuration from plain ROS parameters.

The reasoning is recorded in [`docs/adr/`](https://github.com/minipada/ros2_data_collection/tree/jazzy/docs/adr):
ADR-0001 (external shipper replaces embedded Fluent Bit), ADR-0002 (Vector as the
default shipper), ADR-0003 (blessed Destinations + passthrough), ADR-0005 (file uploads
are a Bridge responsibility), ADR-0006 (the Bridge lives outside the lifecycle manager)
and ADR-0007 (the Bridge is C++).

```admonish info
Nothing in the **collection** half of DC changed. Measurement plugins, Conditions,
Groups, JSON schema validation, and the `dc_interfaces/StringStamped` message are the
same. Everything below is about the **delivery** half.
```

## At a glance

| DC 1.x (Humble)                                       | DC 2.0 (Jazzy)                                                                 |
| ----------------------------------------------------- | ------------------------------------------------------------------------------ |
| `destination_server` node                             | `dc_bridge` node                                                               |
| `destination_plugins: [...]`                          | `destinations: [...]`                                                          |
| `plugin: "dc_destinations/FlbPgSQL"`                  | `type: postgres`                                                               |
| 13 pluginlib destination plugins                      | 4 **blessed** types (`postgres`, `s3`, `file`, `console`) + **passthrough**    |
| Embedded Fluent Bit (forked, built from source)       | External Vector binary, vendored by `vector_vendor`, supervised by the Bridge  |
| `flb.*` service parameters                            | `shipper.data_dir` / `shipper.buffer_max_bytes` (Vector's own defaults for the rest) |
| `tags: [...]` on Measurements/Groups selects outputs  | A Destination's `inputs: [...]` topic list selects what it receives            |
| `flb_minio` + `flb_files_metrics` to upload Files     | One `receives: files` Destination + one `files:` block (the Bridge's Uploader) |
| `src_fields` / `upload_fields` field lists            | Gone — the Uploader reads `local_paths` / `remote_paths` from the Record       |
| A destination not in the list needs a new C++ plugin  | Any Vector sink via `custom_config_files` — no DC code                         |
| Fluent Bit Go plugins (`out_minio.so`, `plugin_path`) | Gone — no Go toolchain, no `plugin_path`                                       |

## The five edits that cover most configurations

1. Rename the node: `destination_server:` → `dc_bridge:`.
2. Rename the list: `destination_plugins:` → `destinations:`.
3. Replace each plugin's `plugin: "dc_destinations/Flb…"` with `type: <blessed type>`
   and add `receives: records` (or `receives: files`, see [Files](#files-flb_minio--flb_files_metrics)).
4. Delete every `tags:` entry from `measurement_server` and `group_server`, and delete
   the whole `flb:` block. Routing is now the Destination's `inputs:` list.
5. Add a `shipper:` block with a `data_dir` for Vector's disk buffer.

A minimal before/after:

```yaml
# DC 1.x
destination_server:
  ros__parameters:
    flb:
      flush: "1"
      storage_path: "/var/log/flb-storage/"
      storage_sync: "full"
      scheduler_cap: "2000"
    destination_plugins: ["flb_stdout"]
    flb_stdout:
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/measurement/uptime"]
measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
      tags: ["flb_stdout"]
```

```yaml
# DC 2.0
dc_bridge:
  ros__parameters:
    shipper:
      data_dir: "$HOME/.dc/buffer"
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
```

## Tags: what changed

In DC 1.x, a Measurement's `tags` parameter named the destination plugins that should
receive its Records, and the embedded engine matched the two.

In DC 2.0 a **Tag** is derived mechanically from the topic a Record was published on —
the leading `/` is dropped and the remaining `/` become `.`, so
`/dc/measurement/uptime` carries the Tag `dc.measurement.uptime`. Every Tag is exposed
on the Shipper as the public route `dc.<tag>` (see
[the routing contract](./destinations.md#the-dctag-routing-contract-public-api)), and a
Destination subscribes by listing the **topics** it wants in `inputs`.

Consequences for a migrated configuration:

- **Delete `tags:`** from every Measurement and Group. It no longer selects a
  Destination. (The parameter is still read and, if non-empty, still written into the
  Record's JSON as a `tags` field — harmless, but it is data, not routing.)
- **A Destination's `inputs` is the routing decision.** To send a Measurement's Records
  to two Destinations, list its `topic_output` in both Destinations' `inputs`.
- **`remote_keys` on file-producing Measurements must be renamed to the Destination
  name.** See below.

## Destination-by-destination reference

The `...` in the "before" snippets stands for the surrounding `destination_server:` /
`ros__parameters:` block; the "after" snippets sit under `dc_bridge:` /
`ros__parameters:`. Destination names (`console`, `pgsql`, `rustfs`, …) are yours to
choose — only `type` is fixed.

### `flb_stdout` → blessed `console`

```yaml
# Before
flb_stdout:
  plugin: "dc_destinations/FlbStdout"
  inputs: ["/dc/group/data"]
  format: "json"
  json_date_key: "date"
  json_date_format: "iso8601"
```

```yaml
# After
destinations: ["console"]
console:
  type: console
  receives: records
  inputs: ["/dc/group/data"]
  time_key: "date"          # was json_date_key
  time_format: "iso8601"    # was json_date_format; "epoch_nanos" (default) | "iso8601" | "double"
```

`format` is gone: the `console` Destination always writes JSON.

### `rcl` → blessed `console`

The `rcl` plugin re-published Records through ROS 2 logging. There is no equivalent —
Records leave the ROS graph at the Bridge. Use the `console` Destination above; it
writes to the Bridge's stdout, which your ROS 2 launch and journald capture already.

### `flb_pgsql` → blessed `postgres`

```yaml
# Before
flb_pgsql:
  plugin: "dc_destinations/FlbPgSQL"
  inputs: ["/dc/group/data"]
  host: "127.0.0.1"
  port: 5432
  user: fluentbit
  password: password
  database: "fluentbit"
  table: "dc"
  timestamp_key: "date"
  async: false
  time_format: "double"
  time_key: "date"
```

```yaml
# After
destinations: ["pgsql"]
pgsql:
  type: postgres
  receives: records
  inputs: ["/dc/group/data"]
  host: "127.0.0.1"
  port: 5432
  user: "dc"
  password: "$DC_PG_PASSWORD"   # $VAR / ${VAR} are expanded from the environment
  database: "dc"
  table: "dc"
  time_key: "date"              # was timestamp_key *and* time_key; now one field
  time_format: "double"
```

`async`, `min_pool_size`/`max_pool_size` and the other connection-pool knobs are gone —
Vector manages its own connections. Note that the Shipper's `postgres` sink maps each
Record's **top-level JSON keys onto existing columns**; it does not create tables or
columns. Create the table up front (see
[`tools/infrastructure/docker/config/postgresql/init.sql`](https://github.com/minipada/ros2_data_collection/blob/jazzy/tools/infrastructure/docker/config/postgresql/init.sql)
for the demo schema).

### `flb_s3` → blessed `s3`

```yaml
# Before
flb_s3:
  plugin: "dc_destinations/FlbS3"
  inputs: ["/dc/group/data"]
  bucket: my-bucket
  region: us-west-2
  total_file_size: "50M"
  use_put_object: false
  compression: "gzip"
  s3_key_format: "/$TAG/%Y/%m/%d/%H_%M_%S.gz"
```

```yaml
# After
destinations: ["s3"]
s3:
  type: s3
  receives: records
  inputs: ["/dc/group/data"]
  bucket: "my-bucket"
  region: "us-west-2"
  key_prefix: "records/"       # replaces s3_key_format's directory part
  batch_timeout_secs: 60       # object write interval; Vector's default is 300
  # access_key_id / secret_access_key omitted: ambient AWS credentials are used
```

`total_file_size`, `use_put_object`, `compression` and `s3_key_format` have no DC
parameter — Vector's `aws_s3` sink owns batching, compression and key naming. If you
need to override its defaults, express the sink through the
[passthrough](#anything-else-passthrough) instead.

### `flb_file` → blessed `file`

```yaml
# Before
flb_file:
  plugin: "dc_destinations/FlbFile"
  inputs: ["/dc/group/memory_uptime"]
  path: "$HOME/data"
  file: "uptime"
  format: "out_file"
  mkdir: true
```

```yaml
# After
destinations: ["local_log"]
local_log:
  type: file
  receives: records
  inputs: ["/dc/group/memory_uptime"]
  path: "$HOME/data/uptime-%Y-%m-%d.log"   # one field; Vector template syntax allowed
```

`path` and `file` merge into a single `path`, which accepts Vector's template syntax
(strftime specifiers, `{{ field }}` interpolation) so one Destination can fan out across
files. Parent directories are always created (`mkdir` is gone), and the output is always
newline-delimited JSON (`format`, `delimiter`, `label_delimiter` are gone).

### Files: `flb_minio` + `flb_files_metrics`

This is the one migration that collapses two plugins, two copies of the same
credentials, and four hand-maintained field lists into a single Destination.

In DC 1.x, uploading a camera image or a map meant:

- `flb_minio` (a Go plugin loaded from `plugin_path`) to push the bytes, configured with
  `src_fields` (where local paths live in the Record) and `upload_fields` (where remote
  paths live);
- `flb_files_metrics` (a second Go plugin) to record upload status in PostgreSQL,
  configured with the **same** object-store credentials, the **same** field lists, plus
  its own `pgsql.*` block;
- and `flb_pgsql` for the Records themselves.

```yaml
# Before
destination_plugins: ["flb_minio", "flb_files_metrics", "flb_pgsql"]
flb_minio:
  plugin: "dc_destinations/FlbMinIO"
  inputs: ["/dc/measurement/camera", "/dc/measurement/map"]
  plugin_path: "/root/ws/src/ros2_data_collection/dc_destinations/flb_plugins/lib/out_minio.so"
  endpoint: 127.0.0.1:9000
  access_key_id: XEYqG4ZcPY5jiq5i
  secret_access_key: ji011KCtI82ZeQS6UwsQAg8x9VR4lSaQ
  use_ssl: false
  create_bucket: true
  bucket: "mybucket"
  src_fields: ["local_paths.inspected", "local_paths.pgm", "local_paths.yaml"]
  upload_fields:
    ["remote_paths.minio.inspected", "remote_paths.minio.pgm", "remote_paths.minio.yaml"]
flb_files_metrics:
  plugin: "dc_destinations/FlbFilesMetrics"
  inputs: ["/dc/measurement/camera", "/dc/measurement/map"]
  file_storage: ["minio"]
  db_type: "pgsql"
  delete_when_sent: true
  minio:
    endpoint: 127.0.0.1:9000
    access_key_id: XEYqG4ZcPY5jiq5i
    secret_access_key: ji011KCtI82ZeQS6UwsQAg8x9VR4lSaQ
    use_ssl: false
    bucket: "mybucket"
    src_fields: ["local_paths.inspected", "local_paths.pgm", "local_paths.yaml"]
    upload_fields:
      ["remote_paths.minio.inspected", "remote_paths.minio.pgm", "remote_paths.minio.yaml"]
  pgsql:
    host: "127.0.0.1"
    port: "5432"
    user: fluentbit
    password: password
    database: "fluentbit"
    table: "files_metrics"
    ssl: false
```

```yaml
# After
destinations: ["pgsql", "pgsql_files", "rustfs"]
pgsql:                                 # the Records themselves
  type: postgres
  receives: records
  inputs: ["/dc/measurement/camera", "/dc/measurement/map"]
  host: "127.0.0.1"
  port: 5432
  user: "dc"
  password: "$DC_PG_PASSWORD"
  database: "dc"
  table: "dc"
pgsql_files:                           # the per-File status log (was files_metrics)
  type: postgres
  receives: records
  # no `inputs`: fed only by files.metadata_destination below
  host: "127.0.0.1"
  port: 5432
  user: "dc"
  password: "$DC_PG_PASSWORD"
  database: "dc"
  table: "dc_files"
rustfs:                                # the File bytes (was flb_minio)
  type: s3
  receives: files
  inputs: ["/dc/measurement/camera", "/dc/measurement/map"]
  bucket: "dc-files"
  endpoint: "http://127.0.0.1:9000"
  region: "us-east-1"
  access_key_id: "rustfsadmin"
  secret_access_key: "$DC_S3_SECRET"
  force_path_style: true
files:
  delete_when_sent: true
  metadata_destination: "pgsql_files"
```

What disappeared and why:

| DC 1.x                                          | DC 2.0                                                                                     |
| ----------------------------------------------- | -------------------------------------------------------------------------------------------|
| `plugin_path` + a Go build                      | Nothing — the Uploader is C++ inside `dc_bridge`                                           |
| `src_fields` / `upload_fields`                  | Nothing — the Uploader reads `local_paths` / `remote_paths` straight from the Record       |
| `file_storage: ["minio", "s3"]`                 | Every `receives: files` Destination is used; the Record's `remote_paths` keys pick which   |
| A second copy of the object-store credentials   | One Destination block, used for both the upload and the status log                         |
| `db_type` + a `pgsql:` sub-block                | `files.metadata_destination`, naming an ordinary `receives: records` Destination           |
| `use_ssl: false`                                | The scheme in `endpoint` (`http://` vs `https://`)                                         |
| `create_bucket: true`                           | Nothing — create the bucket out-of-band (the demo compose file does this with a one-shot container) |
| `ignored` column in `files_metrics`             | Dropped; the status log is append-only (a deletion appends `deleted: true` instead of updating) |

````admonish warning title="Rename remote_keys to the Destination name"
The Uploader sends a File to the `receives: files` Destination whose **name is a key in
the Record's `remote_paths`**. File-producing Measurements build those keys from their
`remote_keys` parameter, which in DC 1.x conventionally said `minio`:

```yaml
# Before
map:
  plugin: "dc_measurements/Map"
  remote_keys: ["minio"]     # produced remote_paths.minio.{yaml,pgm}

# After
map:
  plugin: "dc_measurements/Map"
  remote_keys: ["rustfs"]    # must equal the Destination name declared on dc_bridge
```

If the two disagree, the Files are collected but never uploaded, and no status Record is
emitted.
````

Two guarantees the Uploader adds over `flb_files_metrics`: uploads are **multipart and
resumable** (an interrupted transfer resumes from the last completed part), and a
`kind: group_complete` marker Record is emitted only once **every** File referenced by a
Record has been verified on **every** Destination that receives it. See
[File uploads](./destinations.md#file-uploads-receives-files-the-uploader-adr-0005) for
the full contract.

```admonish info title="MinIO to RustFS"
MinIO's community edition was archived upstream in 2026. DC 2.0's demos and
infrastructure files use [RustFS](https://rustfs.com/) (Apache 2.0, S3-compatible)
instead. This is a *recommendation*, not a requirement — the blessed `s3` type talks to
MinIO, Ceph RGW, or AWS S3 with the same parameters.
```

### `flb_http` → passthrough `http`

```yaml
# Before
flb_http:
  plugin: "dc_destinations/FlbHTTP"
  inputs: ["/dc/measurement/data"]
  host: "127.0.0.1"
  port: 80
  uri: "/"
  format: "json"
```

```yaml
# After — dc_bridge parameters
destinations: ["console"]              # keep at least one blessed Destination, or none
custom_config_files: ["$HOME/.dc/http_sink.toml"]
```

```toml
# After — $HOME/.dc/http_sink.toml
[sinks.my_http]
type = "http"
inputs = ["dc.dc.measurement.data"]    # the public dc.<tag> route for that topic
uri = "http://127.0.0.1:80/"
encoding.codec = "json"
```

`http_user`/`http_passwd` become Vector's `auth.strategy = "basic"` block; `header`
entries become a `[sinks.my_http.request.headers]` table.

### `flb_influxdb` → passthrough `influxdb_logs`

```yaml
# Before
flb_influxdb:
  plugin: "dc_destinations/FlbInfluxDB"
  inputs: ["/dc/measurement/uptime"]
  host: "127.0.0.1"
  port: 8086
  database: "ros"
```

```toml
# After — a passthrough snippet listed in custom_config_files
[sinks.influxdb]
type = "influxdb_logs"
inputs = ["dc.dc.measurement.uptime"]
endpoint = "http://127.0.0.1:8086"
measurement = "dc"

[sinks.influxdb.influxdb1_settings]
database = "ros"
```

A complete, runnable version of this ships with the demos —
[`dc_demos/config/tb3_simulation_influxdb_sink.toml`](https://github.com/minipada/ros2_data_collection/blob/jazzy/dc_demos/config/tb3_simulation_influxdb_sink.toml),
used by the [InfluxDB demo](./demos/tb3_aws_influxdb.md). For InfluxDB 2.x use
`influxdb2_settings` with `org`, `bucket` and `token` instead.

### `flb_kinesis_streams` → passthrough `aws_kinesis_streams`

```yaml
# Before
flb_kinesis_streams:
  plugin: "dc_destinations/FlbKinesisStreams"
  inputs: ["/dc/measurement/data"]
  region: "us-east-1"
  stream: my_stream
```

```toml
# After
[sinks.kinesis]
type = "aws_kinesis_streams"
inputs = ["dc.dc.measurement.data"]
region = "us-east-1"
stream_name = "my_stream"
partition_key_field = "name"
encoding.codec = "json"
```

`role_arn` becomes Vector's `auth.assume_role`.

### `flb_slack` → passthrough `http`

Vector has no dedicated Slack sink; post to the same incoming webhook with the `http`
sink.

```yaml
# Before
flb_slack:
  plugin: "dc_destinations/FlbSlack"
  inputs: ["/dc/group/data"]
  webhook: https://hooks.slack.com/services/T00000000/B00000000/XXXXXXXXXXXXXXXXXXXXXXXX
```

```toml
# After
[transforms.slack_payload]
type = "remap"
inputs = ["dc.dc.group.data"]
source = '. = { "text": encode_json(.) }'   # Slack wants {"text": "..."}

[sinks.slack]
type = "http"
inputs = ["slack_payload"]
uri = "$SLACK_WEBHOOK_URL"
encoding.codec = "json"
```

### `flb_tcp` → passthrough `socket`

```yaml
# Before
flb_tcp:
  plugin: "dc_destinations/FlbTCP"
  inputs: ["/dc/measurement/uptime"]
  host: "127.0.0.1"
  port: 5170
```

```toml
# After
[sinks.tcp]
type = "socket"
inputs = ["dc.dc.measurement.uptime"]
mode = "tcp"
address = "127.0.0.1:5170"
encoding.codec = "json"
```

### `flb_null` → delete it, or passthrough `blackhole`

`flb_null` discarded Records. In DC 2.0 a topic that is not listed in any Destination's
`inputs` is simply never routed anywhere, so the usual migration is to **delete the
destination entirely**. If you were using it to exercise the pipeline under load, the
equivalent is Vector's `blackhole` sink:

```toml
[sinks.devnull]
type = "blackhole"
inputs = ["dc.dc.measurement.data"]
```

### Anything else: passthrough

Any sink in [Vector's catalog](https://vector.dev/docs/reference/configuration/sinks/)
— Kafka, Elasticsearch, Loki, Datadog, ClickHouse, GCP, Azure … — is reachable the same
way, with no DC code and no new language toolchain. See
[Destinations](./destinations.md#passthrough-custom_config_files) for the rules
(snippets must not redefine component ids the generated config owns, and the merged set
is checked with `vector validate` before the Shipper starts), and the
[Elasticsearch tutorial](./demos/elasticsearch.md) for a worked end-to-end example —
including the two things a snippet has to do for itself that a blessed Destination does
for you (its own disk buffer, and idempotent re-delivery).

## Service-level parameters

The `flb:` block configured the embedded Fluent Bit engine. Vector's equivalents are
either its own defaults or a `shipper:` parameter on the Bridge.

| DC 1.x `flb.*`                                                       | DC 2.0                                                                     |
| -------------------------------------------------------------------- | ---------------------------------------------------------------------------|
| `storage_path`                                                       | `shipper.data_dir` (default `$HOME/.dc/buffer`)                            |
| `storage_backlog_mem_limit`                                          | `shipper.buffer_max_bytes` (Vector's minimum is ~256 MiB)                  |
| `flush`, `grace`                                                     | Vector's own flush/shutdown behaviour; no DC parameter                     |
| `log_level`                                                          | Vector's `VECTOR_LOG` environment variable                                 |
| `storage_sync`, `storage_checksum`, `in_storage_type`                | Vector's disk buffers are always durable and checksummed; no DC parameter  |
| `scheduler_cap`, `scheduler_base`                                    | Vector's per-sink retry backoff; no DC parameter                           |
| `http_server`, `metrics`                                             | Vector's `internal_metrics` source via the passthrough                     |
| `in_storage_pause_on_chunks_overlimit`                               | Vector applies backpressure to the Bridge instead; no DC parameter         |

Two Bridge parameters replace the implicit in-process connection between the measurement
node and the embedded engine:

```yaml
dc_bridge:
  ros__parameters:
    vector_forward_host: "127.0.0.1"   # where the Shipper listens
    vector_forward_port: 24224
```

## Build and runtime changes

- **No Fluent Bit fork.** DC 1.x built a patched Fluent Bit (and, before #235, needed
  root to run it). `fluent_bit_plugins` and `dc_destinations` no longer exist.
- **No Go toolchain.** The `out_minio.so` / `out_files_metrics.so` Go plugins are gone
  along with `plugin_path`.
- **Vector is vendored.** `vector_vendor` downloads a pinned, checksummed Vector binary
  at build time. Point `-Dvector_path=/usr/bin/vector` (or `VECTOR_PATH`) at a
  system-installed binary for air-gapped builds.
- **Startup is ordered.** The Bridge starts before the collection nodes and a readiness
  gate blocks activation until the Shipper is accepting connections, so no Record is
  produced before the pipeline can accept it. See
  [Data Pipeline](./data_pipeline.md).

See [Setup](./setup.md) for the resulting install, which is now `rosdep install` +
`colcon build`.

## Timestamps: `time_format` now defaults to `epoch_nanos`

Records carry their ROS header timestamp at full nanosecond resolution, and the default
`time_format` writes it as an exact integer count of nanoseconds since the epoch rather
than fractional seconds in a float64.

```admonish warning title="Schema change"
The column receiving `time_key` must be **`bigint`**, not `double precision`. A double
rounds the value back off below roughly microsecond resolution — which is what stopped
timestamps telling two Records of a fast Measurement apart in the first place.

If you already have a populated `dc_records` from an earlier `jazzy` checkout:

    ALTER TABLE dc_records ALTER COLUMN date TYPE bigint USING (date * 1e9)::bigint;

and update any query that treated the column as seconds — `to_timestamp(date)` becomes
`to_timestamp(date / 1e9)`.
```

Set `time_format: "double"` explicitly to keep the old fractional-seconds column. It
remains supported, and remains lossy: a float64 has ~15-16 significant digits and current
epoch seconds already spend 10 of them. `iso8601` is the other lossless option.

## `custom_str_params*` renamed to `custom_keys_str*`

`measurement_server`'s custom-key parameters were named `custom_str_params_list` /
`custom_str_params.*`, even though what they add are custom **keys** in the Record JSON,
not parameters of the node. Rename them:

| DC 1.x / earlier `jazzy`         | DC 2.0                       |
| --------------------------------- | ----------------------------- |
| `custom_str_params_list`          | `custom_key_str_list`         |
| `custom_str_params.<name>`        | `custom_keys_str.<name>`      |
| `custom_str_params.force_override`| `custom_keys_str.force_override` |

```yaml
# Before
measurement_server:
  ros__parameters:
    custom_str_params_list: ["robot_name"]
    custom_str_params:
      robot_name:
        name: robot_name
        value: "C3PO"

# After
measurement_server:
  ros__parameters:
    custom_key_str_list: ["robot_name"]
    custom_keys_str:
      robot_name:
        name: robot_name
        value: "C3PO"
```

## Migration checklist

- [ ] `destination_server:` renamed to `dc_bridge:`, `destination_plugins:` to `destinations:`
- [ ] The column behind `time_key` is `bigint` (or `time_format: "double"` is set explicitly)
- [ ] Every destination has a `type:` and a `receives:` instead of a `plugin:`
- [ ] The `flb:` block is deleted; `shipper.data_dir` is set
- [ ] Every `tags:` entry under `measurement_server` / `group_server` is deleted
- [ ] Every Destination lists the topics it should receive in `inputs:`
- [ ] File-producing Measurements' `remote_keys` match a `receives: files` Destination name
- [ ] `files.metadata_destination` names a `receives: records` Destination
- [ ] Un-blessed destinations are expressed as `custom_config_files` snippets consuming `dc.<tag>` routes
- [ ] Destination tables exist in PostgreSQL, and object-storage buckets are created
- [ ] `custom_str_params_list:` / `custom_str_params:` under `measurement_server` are renamed to `custom_key_str_list:` / `custom_keys_str:`
- [ ] Secrets moved out of the params file into `$VAR` environment references
