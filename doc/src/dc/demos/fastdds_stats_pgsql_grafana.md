# Fast DDS statistics to PostgreSQL/Grafana

```admonish warning title="Fast DDS-specific"
This demo only produces data when Fast DDS is the RMW in use **and** it was built with its
Statistics Module enabled (`-DFASTDDS_STATISTICS=ON`), plus `fastdds_statistics_backend`
installed against it — pin the `v1.1.0` tag; `v2.0.0` onward requires Fast-DDS 3.0.0, which
ROS 2 Jazzy's 2.14.x doesn't satisfy. A third, runtime-only prerequisite is easy to miss: the
`FASTDDS_STATISTICS` environment variable must be set on every process *before* it creates its
first DomainParticipant, or `latency_ns_mean` and every throughput/RTPS field stay permanently
absent even though the plugin itself runs fine. See [the Fast DDS statistics Measurement
doc](../measurements/fastdds_stats.md) for the full build recipe and all three prerequisites.
Without the first two, `dc_measurements` still builds and every other demo still runs —
`dc_measurements/CMakeLists.txt` finds `fastdds_statistics_backend` optionally and skips only
this one plugin — but launching *this* demo fails: `measurement_server` can't load a
`dc_measurements/FastddsStats` plugin that was never built.
```

This is the smallest hardware-free way to see Fast DDS's own Statistics Module land in DC: one
Measurement ([Fast DDS statistics](../measurements/fastdds_stats.md)), a passthrough `postgres`
sink ([ADR-0003](../adr/0003-blessed-destinations-plus-passthrough.md)), and a Grafana dashboard
provisioned automatically — the same convention #304 established for the
[KPI dashboard](../kpi_views.md).

## Setup Infrastructure

### PostgreSQL

[Follow the steps](../infrastructure_setup/postgresql.md) to start it. The default yaml
configuration file does not need change.

### Grafana

[Follow the steps](../infrastructure_setup/grafana.md) to start it.

## Run the demo

```bash
colcon build
mkdir -p ~/.dc && cp "$(ros2 pkg prefix dc_demos)/share/dc_demos/config/fastdds_stats_pgsql_grafana_sink.toml" ~/.dc/
export FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;SUBSCRIPTION_THROUGHPUT_TOPIC;RTPS_SENT_TOPIC;RTPS_LOST_TOPIC"
ros2 launch dc_demos fastdds_stats_pgsql_grafana.launch.py
```

A Record captured from a real run, echoed straight off `/dc/measurement/fastdds_stats`
(no PostgreSQL needed to see this — it's what `dc_bridge` forwards on):

```json
{
  "custom_keys": ["robot_name"],
  "datareader_count": 1,
  "datawriter_count": 11,
  "domain_id": 0,
  "event": "sample",
  "flattened": false,
  "hosts": ["d:14058711922191368192"],
  "name": "fastdds_stats",
  "nested": false,
  "participant_count": 3,
  "participants": [
    { "guid": "01.0f.4d.26.9c.1d.72.46.00.00.00.00|0.0.1.c1", "name": "/" },
    { "guid": "01.0f.4d.26.13.27.ae.cb.00.00.00.00|0.0.1.c1", "name": "/" },
    { "guid": "01.0f.4d.26.25.27.c4.fd.00.00.00.00|0.0.1.c1", "name": "/" }
  ],
  "process_names": ["7580", "10003", "10021"],
  "robot_name": "C3PO",
  "run_id": "172",
  "users": ["root"]
}
```

`latency_ns_mean` and the throughput/RTPS fields are absent in this particular sample:
nothing exchanged data on a matched DataWriter/DataReader pair within that 5-second poll
window. See [the Measurement's own page](../measurements/fastdds_stats.md) for why
absence is what "nothing to report" looks like here.

## Visualize the data

Open [http://localhost:3000](http://localhost:3000) (admin/admin) and pick the **ROS 2 Data
Collection - Fast DDS statistics** dashboard. Its panels are backed by SQL queries against the
`dc` PostgreSQL table (datasource uid `dc_postgres`), the same one every other PostgreSQL demo
writes to — filtered to `WHERE name = 'fastdds_stats'`:

- Write-to-notification latency, mean across matched DataWriter/DataReader pairs
- Publication and subscription throughput, mean
- Discovered participant/DataWriter/DataReader counts over time
- RTPS packets sent and lost
- The most recent sample's physical-layer data — which hosts, users and processes are behind the
  discovered participants

See `tools/infrastructure/docker/config/grafana/dashboards/fastdds_stats.json` for every panel's
exact query.

## Understanding the configuration

```admonish info
The full configuration file can be found [here](https://github.com/Minipada/ros2_data_collection/blob/jazzy/dc_demos/params/fastdds_stats_pgsql_grafana.yaml).
```

```yaml
measurement_server:
  ros__parameters:
    measurement_plugins: ["fastdds_stats"]
    fastdds_stats:
      plugin: "dc_measurements/FastddsStats"
      topic_output: "/dc/measurement/fastdds_stats"
      polling_interval: 5000
      domain_id: 0
      include_measurement_name: true
      init_collect: true

dc_bridge:
  ros__parameters:
    destinations: ["records_log"]
    records_log:
      type: file
      receives: records
      inputs: ["/dc/measurement/fastdds_stats"]
      path: "/tmp/dc/fastdds_stats_pgsql_grafana_records.ndjson"
      time_key: "date"
      time_format: "double"
    custom_config_files: ["$HOME/.dc/fastdds_stats_pgsql_grafana_sink.toml"]
```

```toml
# ~/.dc/fastdds_stats_pgsql_grafana_sink.toml
[sinks.pgsql]
type = "postgres"
inputs = ["dc.dc.measurement.fastdds_stats"]
endpoint = "postgres://dc:password@127.0.0.1:5432/dc"
table = "dc"

[sinks.pgsql.buffer]
type = "disk"
max_size = 268435488
```

`postgres` — along with `s3` and `console` — moved from the blessed ROS-param form to this
passthrough recipe ([ADR-0003](../adr/0003-blessed-destinations-plus-passthrough.md)); `records_log`
is the cheap `file` anchor the passthrough still needs, since `dc_bridge` derives its ROS
subscriptions and `dc.<tag>` routes from `destinations` alone. See
[Destinations: Recipes](../destinations.md#recipes-postgres-s3-console-via-passthrough) for the
full recipe. `domain_id` is the DDS domain to monitor — the same value `ROS_DOMAIN_ID` would use
for every other node in the deployment. `include_measurement_name: true` writes
`"name": "fastdds_stats"` onto every Record, which every panel's `WHERE name = 'fastdds_stats'`
clause relies on to tell this Measurement's rows apart from any other demo sharing the same `dc`
table.
