# Fast DDS statistics to PostgreSQL/Grafana

```admonish warning title="Fast DDS-specific"
This demo only produces data when Fast DDS is the RMW in use **and** it was built with its
Statistics Module enabled (`-DFASTDDS_STATISTICS=ON`), plus `fastdds_statistics_backend`
installed. See [the Fast DDS statistics Measurement doc](../measurements/fastdds_stats.md) for
both prerequisites. Without them, `dc_measurements` still builds and every other demo still runs
— `dc_measurements/CMakeLists.txt` finds `fastdds_statistics_backend` optionally and skips only
this one plugin — but launching *this* demo fails: `measurement_server` can't load a
`dc_measurements/FastddsStats` plugin that was never built.
```

This is the smallest hardware-free way to see Fast DDS's own Statistics Module land in DC: one
Measurement ([Fast DDS statistics](../measurements/fastdds_stats.md)), a `postgres` Destination,
and a Grafana dashboard provisioned automatically — the same convention #304 established for the
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
ros2 launch dc_demos fastdds_stats_pgsql_grafana.launch.py
```

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
    destinations: ["pgsql"]
    pgsql:
      type: postgres
      receives: records
      inputs: ["/dc/measurement/fastdds_stats"]
      host: "127.0.0.1"
      port: 5432
      user: "dc"
      password: "password"
      database: "dc"
      table: "dc"
      time_key: "date"
      time_format: "double"
```

`domain_id` is the DDS domain to monitor — the same value `ROS_DOMAIN_ID` would use for every
other node in the deployment. `include_measurement_name: true` writes `"name": "fastdds_stats"`
onto every Record, which every panel's `WHERE name = 'fastdds_stats'` clause relies on to tell
this Measurement's rows apart from any other demo sharing the same `dc` table.
