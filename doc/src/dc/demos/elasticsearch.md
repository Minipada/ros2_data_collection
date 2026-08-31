# Elasticsearch (passthrough)

`dc_bridge` blesses exactly five Destination types — `postgres`, `s3`, `file`, `console`,
`vector` (see [Destinations](../destinations.md)) — and Elasticsearch is not one of them. This
tutorial is the worked example for reaching everything else: the ADR-0003 **passthrough**,
a raw [Vector](https://vector.dev) config snippet loaded through `custom_config_files`
that consumes the same public `dc.<tag>` routes a blessed Destination consumes. Nothing in
`dc_bridge` knows what Elasticsearch is, and no DC code was written to support it.

Elasticsearch is the example because it is the one most often asked for, but the shape
generalises: swap the sink type and you have Kafka, Loki, ClickHouse, Datadog, or anything
else in [Vector's catalog](https://vector.dev/docs/reference/configuration/sinks/). The
[InfluxDB demo](./tb3_aws_influxdb.md) is the same mechanism against a Turtlebot3
simulation; this page needs no simulator and no robot — four system Measurements, one
compose file, one terminal.

```admonish info
Verified end to end against Elasticsearch 8.19.5 and the Vector 0.57.0 binary vendored by
`vector_vendor`. Version-sensitive details are called out where they matter.
```

## Setup the infrastructure

Start Elasticsearch and Kibana by [following the steps](../infrastructure_setup/elasticsearch.md):

```bash
podman compose -f tools/infrastructure/docker/docker-compose.elasticsearch.yaml up -d
```

Confirm the cluster is up before launching DC — the sink retries a store that isn't there
yet, but a `green` cluster makes the first Record land immediately:

```bash
curl -s http://localhost:9200/_cluster/health
```

## One-time: install the passthrough sink config

Because there is no Elasticsearch Destination to configure through ROS parameters, the
Vector sink itself ships as a plain file in this package,
`dc_demos/config/elasticsearch_sink.toml`, installed to the package's share directory.
Copy it into place once before the first launch:

```bash
mkdir -p ~/.dc
cp "$(ros2 pkg prefix dc_demos)/share/dc_demos/config/elasticsearch_sink.toml" ~/.dc/
```

The params file's `custom_config_files` points at that path. Editing the copy in `~/.dc/`
is how you change the sink — it is read at Bridge startup, not compiled in.

## Run it

```bash
colcon build
ros2 launch dc_demos elasticsearch.launch.py
```

The `console` Destination prints every Record as it is shipped, so the terminal doubles as
a local view of what Elasticsearch is receiving:

```
[dc_bridge-2] {"cpu":{"average":23.25,"processes":4,"sorted":[]},"date":1786118523.0,"flattened":false,"host":"127.0.0.1","name":"cpu","nested":true,"robot_name":"C3PO","run_id":"1","source_type":"fluent","tag":"dc.measurement.cpu","timestamp":"2026-08-07T16:02:03Z"}
[dc_bridge-2] {"date":1786118523.0,"flattened":false,"host":"127.0.0.1","memory":{"used":97.25801086425781},"name":"memory","nested":true,"robot_name":"C3PO","run_id":"1","source_type":"fluent","tag":"dc.measurement.memory","timestamp":"2026-08-07T16:02:03Z"}
[dc_bridge-2] {"date":1786118523.0,"flattened":false,"host":"127.0.0.1","name":"uptime","nested":true,"robot_name":"C3PO","run_id":"1","source_type":"fluent","tag":"dc.measurement.uptime","timestamp":"2026-08-07T16:02:03Z","uptime":{"time":371820}}
```

## Visualize the data

### With `curl`

The sink writes one index per UTC day, so `dc-records-*` is every day's data:

```bash
curl -s 'http://localhost:9200/dc-records-*/_count'
```

```json
{"count":25,"_shards":{"total":1,"successful":1,"skipped":0,"failed":0}}
```

Count the documents per Measurement — `cpu`, `memory` and `uptime` poll every 5 seconds,
while `os` is configured to collect once (`init_max_measurements: 1`), which is exactly
what shows up:

```bash
curl -s -H 'Content-Type: application/json' 'http://localhost:9200/dc-records-*/_search' \
  -d '{"size":0,"aggs":{"by_name":{"terms":{"field":"name.keyword"}}}}'
```

```json
"buckets": [
  { "key": "cpu",    "doc_count": 8 },
  { "key": "memory", "doc_count": 8 },
  { "key": "uptime", "doc_count": 8 },
  { "key": "os",     "doc_count": 1 }
]
```

And one document in full:

```bash
curl -s -H 'Content-Type: application/json' 'http://localhost:9200/dc-records-*/_search' \
  -d '{"size":1,"query":{"term":{"name.keyword":"os"}}}'
```

```json
{
  "@timestamp": "2026-08-07T15:47:09Z",
  "date": 1786117629.0,
  "flattened": false,
  "host": "127.0.0.1",
  "name": "os",
  "nested": true,
  "os": {
    "cpus": 8,
    "kernel": "6.12.96+deb13-amd64",
    "memory": 23.23,
    "os": "Ubuntu 24.04.4 LTS"
  },
  "robot_name": "C3PO",
  "run_id": "1",
  "source_type": "fluent",
  "tag": "dc.measurement.os",
  "timestamp": "2026-08-07T15:47:09Z"
}
```

The Record's own fields (`os.*`, `date`, `robot_name`, `run_id`) are joined by four the
Shipper adds: `tag` — the [Tag](../destinations.md#the-dctag-routing-contract-public-api)
the Record was routed under, useful as a filter — plus `host`, `source_type` and
`timestamp` from Vector's ingest source. `@timestamp` is added by the snippet; see below.

### With Kibana

Open <http://localhost:5601>, then **Stack Management → Data Views → Create data view**:

| Field           | Value           |
| --------------- | --------------- |
| Name            | `dc`            |
| Index pattern   | `dc-records-*`  |
| Timestamp field | `@timestamp`    |

**Discover** then shows Records as they arrive, and `cpu.average`, `memory.used` and
`uptime.time` are all numeric fields ready to chart in **Lens** — no mapping to declare,
because Elasticsearch inferred it from the first document of each index.

## Understanding the configuration

```admonish info
The full configuration file can be found [here](https://github.com/Minipada/ros2_data_collection/blob/jazzy/dc_demos/params/elasticsearch.yaml), and the passthrough sink config [here](https://github.com/Minipada/ros2_data_collection/blob/jazzy/dc_demos/config/elasticsearch_sink.toml).
```

### Measurements

Four system Measurements, none of which need hardware, a simulator, or a running robot:

1. [CPU](../measurements/cpu.md)
2. [Memory](../measurements/memory.md)
3. [OS](../measurements/os.md)
4. [Uptime](../measurements/uptime.md)

```yaml
measurement_server:
  ros__parameters:
    measurement_plugins: ["cpu", "memory", "os", "uptime"]
    cpu:
      plugin: "dc_measurements/Cpu"
      topic_output: "/dc/measurement/cpu"
      polling_interval: 5000
      include_measurement_name: true
      init_collect: true
      max_processes: 5
      cpu_min: 5.0
      nested: true
      flatten: false
```

The one destination-specific choice here is **`flatten: false`**. With `nested: true` the
Measurement wraps its values under its own key; `flatten` then decides whether that
structure is collapsed into keys like `/cpu/average` (what the InfluxDB and PostgreSQL
demos use, because a line-protocol or column-shaped store wants flat fields) or left as
real nested JSON. Elasticsearch stores JSON natively and maps a nested object to dotted
field names on its own, so leaving it nested gives you `cpu.average` — a field Kibana can
aggregate — instead of a literal field named `/cpu/average`, which every KQL query would
have to escape.

### Destination: what actually creates the routes

This is the part of the passthrough that surprises people:

```yaml
dc_bridge:
  ros__parameters:
    shipper:
      data_dir: "$HOME/.dc/buffer"
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs:
        [
          "/dc/measurement/cpu",
          "/dc/measurement/memory",
          "/dc/measurement/os",
          "/dc/measurement/uptime",
        ]
      time_key: "date"
      time_format: "double"
    custom_config_files: ["$HOME/.dc/elasticsearch_sink.toml"]
    vector_forward_host: "127.0.0.1"
    vector_forward_port: 24224
```

`destinations` still names a blessed Destination, and it is **not** decoration.
`dc_bridge` derives two things from `destinations` and nothing else:

- which ROS topics it subscribes to, and
- which `dc.<tag>` route branches exist in the Vector config it renders.

It never reads a passthrough snippet's `inputs`. So a topic that appears only in the
snippet is a topic the Bridge never subscribes to and never routes — the snippet's
`inputs` would resolve to routes that don't exist. Every topic you want in Elasticsearch
must therefore appear in some blessed Destination's `inputs` too.

`console` is the cheapest thing to put there — it needs no infrastructure and no
credentials — and it earns its place by giving you the local view shown above. A `file`
Destination works equally well if you'd rather not have the output in your terminal.

```admonish warning
`destinations: []` does not work. rclcpp cannot load an empty YAML sequence (it has no
inferable element type), so the Bridge dies at startup with `parameter_value_from failed
for parameter 'destinations': No parameter value set` and never gets as far as reading
`custom_config_files`. A passthrough always accompanies at least one blessed Destination.
```

### The passthrough snippet

```toml
[transforms.elasticsearch_prepare]
type = "remap"
inputs = [
  "dc.dc.measurement.cpu",
  "dc.dc.measurement.memory",
  "dc.dc.measurement.os",
  "dc.dc.measurement.uptime",
]
source = '''
."@timestamp" = .timestamp

.doc_id = sha2(
  to_string!(.tag) + "|" + to_string!(.date) + "|" + to_string!(.run_id),
  variant: "SHA-256"
)
'''

[sinks.elasticsearch]
type = "elasticsearch"
inputs = ["elasticsearch_prepare"]
endpoints = ["http://127.0.0.1:9200"]
api_version = "v8"
mode = "bulk"
bulk.index = "dc-records-%Y.%m.%d"
bulk.action = "index"
id_key = "doc_id"

[sinks.elasticsearch.buffer]
type = "disk"
max_size = 268435488
```

Each `inputs` entry is one of the stable `dc.<tag>` routes. The Tag is the topic name with
the leading `/` dropped and the remaining `/` turned into `.` (`/dc/measurement/cpu` →
`dc.measurement.cpu`), and the route is `dc.<tag>` — so `dc.dc.measurement.cpu`. These
names are public API; see
[Destinations](../destinations.md#the-dctag-routing-contract-public-api).

Note that a snippet is not limited to sinks. `elasticsearch_prepare` is a transform, and
it is merged into the topology like any other component — the only rule is that a snippet
must not *define* a component id the generated config owns (`dc_bridge_in`,
`dc_bridge_normalize`, `dc`, or a configured Destination's name). Consuming those ids is
fine; redefining one is a loud Bridge startup error naming the file.

Four things in there are worth explaining, because three of them are what separate a
snippet that works from a snippet that works *and* survives contact with a real
deployment:

**`."@timestamp" = .timestamp`** — Elasticsearch's convention for a document's time field
is `@timestamp`, and Kibana offers it as the default when you create a data view. Vector
calls it `timestamp`. This copies it across. Copy, not move: `bulk.index` is a strftime
template rendered from the event's own `timestamp`, and deleting that field makes every
document fail to render an index name and get dropped.

**`bulk.index = "dc-records-%Y.%m.%d"`** — one index per UTC day, the usual shape for
time-series data. `dc-records-*` is then a single Kibana data view, and expiring old data
is deleting whole indices rather than running delete-by-query.

**`id_key` + a deterministic `doc_id`** — the Shipper is at-least-once (ADR-0002). After an
outage, Records that were in flight when it started are re-sent on recovery, and with
Elasticsearch's default auto-generated `_id` every re-send becomes a *new document*.
Hashing (Tag, Record timestamp, run id) into a stable id makes the write an upsert instead,
so a re-delivery overwrites its earlier copy. Vector moves the `doc_id` field into the
document's `_id` rather than storing it in the body.

**The disk buffer** — `dc_bridge` gives every *blessed* sink a disk buffer, but a
passthrough sink gets Vector's default, which is in-memory and 500 events deep. That is
not a correctness problem (see below), but a shallow buffer starts applying backpressure
within seconds of a store going away, which is what turns a short outage into a large
burst of re-deliveries. `max_size` is in bytes and Vector rejects anything below ~256 MiB;
the buffer lives under the Bridge's own `shipper.data_dir`.

## What happens when Elasticsearch goes away

Worth doing once, because it is the question every passthrough raises — the Bridge's
durability guarantees are documented for blessed Destinations, and it is reasonable to
wonder whether a sink DC knows nothing about still gets them.

Leave DC running and stop the store for two minutes:

```bash
podman stop elasticsearch
# ...wait...
podman start elasticsearch
```

Vector logs the failure and retries with exponential backoff, so the terminal fills with:

```
WARN sink{component_id=elasticsearch component_type=elasticsearch}: vector::sinks::util::retries: Retrying after error. error=Failed to make HTTP(S) request: error trying to connect: tcp connect error: Connection refused (os error 111)
WARN sink{component_id=elasticsearch component_type=elasticsearch}: vector::sinks::util::service::health: Endpoint is unhealthy. endpoint=http://127.0.0.1:9200
```

`uptime.time` increases by exactly 5 every Record, which makes gaps and duplicates
countable. Across a 2-minute outage, with the sink above, all 110 Records spanning the
outage arrive: none missing, none duplicated.

Two things produced that result, and both are choices in the snippet rather than anything
DC does for you:

- **No loss** comes from the Shipper's end-to-end acknowledgements. `dc_bridge` holds a
  Record until Vector confirms it, so a full buffer means the Bridge retries rather than
  drops — this part you get for free, disk buffer or not.
- **No duplicates** comes from the deterministic `_id`. Running the same outage with
  Vector's default in-memory buffer and Elasticsearch's auto-generated ids also loses
  nothing, but indexes each Record up to a dozen times: backpressure hits within seconds,
  the Bridge re-sends unacknowledged Records, and every re-send lands as its own document.

```admonish info
Recovery is not instant. Vector's retry backoff grows to roughly a minute between
attempts, so after a long outage the backlog starts draining up to a minute after the
store comes back, then catches up quickly.
```

## Adapting it

- **Another store.** Replace `[sinks.elasticsearch]` with any other type from
  [Vector's sink catalog](https://vector.dev/docs/reference/configuration/sinks/). The
  `inputs`, the blessed-Destination requirement, and the at-least-once caveat are the
  same; only the sink block changes.
- **A real robot's data.** Add the topics to both the blessed Destination's `inputs` and
  the snippet's. The [Turtlebot3 AWS Warehouse](./tb3_aws_minio_pgsql.md) params file is a
  good source of Measurement configuration to copy from.
- **A secured cluster.** Add an `auth` block to the sink — there is a commented example at
  the bottom of `elasticsearch_sink.toml`. Vector expands `$VAR` references in its own
  config at startup, so the password can come from the environment.

## Cleanup

```bash
podman compose -f tools/infrastructure/docker/docker-compose.elasticsearch.yaml down -v
```
