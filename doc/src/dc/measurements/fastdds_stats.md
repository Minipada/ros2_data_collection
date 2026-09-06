# Fast DDS statistics

```admonish warning title="Fast DDS-specific"
This Measurement links against eProsima's `Fast-DDS-statistics-backend` C++ library and reads
data Fast DDS's own Statistics Module produces. It only means anything when the deployment runs
**Fast DDS** as its RMW — Cyclone DDS and other RMW implementations have no equivalent library to
read from (checked: `cyclonedds-insight`, eProsima's own recently-announced tool, is GUI-only with
no headless export path). It builds conditionally: `dc_measurements/CMakeLists.txt` looks for
`fastdds_statistics_backend` and skips building this plugin — with a clear `message(STATUS ...)`,
not a failed workspace build — when it isn't found. Configuring
`fastdds_stats: {plugin: "dc_measurements/FastddsStats"}` without the library built fails
pluginlib's load loudly (missing library), rather than silently doing nothing.
```

## Description

Reads eProsima Fast DDS's own Statistics Module — latency, throughput and RTPS packet counts
between the DomainParticipants, DataWriters and DataReaders it discovers on a DDS domain — through
`Fast-DDS-statistics-backend`, and emits one `sample` Record per polling interval. It follows the
same periodic-sample convention as [Battery](./battery.md)'s `sample` event and
[Uptime](./uptime.md): everything reported is a fact for the window since the previous poll, not a
running average since the Measurement started.

Unlike every other Measurement, `fastdds_stats` has no input topic: it starts a
`StatisticsBackend` monitor on a DDS domain at `onConfigure()` and queries that domain's own
statistics registry directly on each poll, rather than subscribing to anything. A robot's whole DDS
graph (every node's participants, on whatever domain it runs) is visible to one Measurement
instance, so normally one `fastdds_stats` Measurement per robot is enough.

Each sample reports:

- Discovered entity counts (`participant_count`, `datawriter_count`, `datareader_count`)
- Mean write-to-notification latency across every matched DataWriter/DataReader pair
  (`latency_ns_mean`, nanoseconds — Fast DDS's own unit for this statistic)
- Mean data rate sent and received (`publication_throughput_bytes_per_sec_mean`,
  `subscription_throughput_bytes_per_sec_mean`, bytes/second)
- RTPS packets sent and lost, summed across every participant (`rtps_packets_sent`,
  `rtps_packets_lost`)
- Physical-layer data: each discovered participant's `name`/`guid`, and the names of the hosts,
  OS users and processes running them (`hosts`, `users`, `process_names` — not `processes`, which
  the [CPU](./cpu.md) Measurement's total process count already owns in the shared `dc` table) —
  useful for spotting which machine or process is actually behind a noisy participant on a
  multi-process robot

A field tied to a DataKind (latency, throughput, packet counts) is **absent**, not zero, when
nothing reported data in the window — a domain with one lonely participant and no matched
DataWriter/DataReader pair yet still produces a valid Record, just without a `latency_ns_mean`.

```admonish info title="What init_monitor(domain_id) does"
`onConfigure()` calls `StatisticsBackend::init_monitor(domain_id)` once, which is what makes Fast
DDS's Statistics Module start reporting for that domain at all — nothing is collected on a domain
no Measurement has called `init_monitor()` for. `onCleanup()` calls `stop_monitor()` to tear it back
down on a lifecycle transition.
```

## Prerequisites

1. Fast DDS itself built with `-DFASTDDS_STATISTICS=ON` — the Statistics Module is compiled out by
   default, and most distro/apt Fast DDS builds do not enable it. ROS 2 Jazzy pairs with Fast-DDS
   2.14.x (`eProsima/Fast-DDS` @ `2.14.x`, `eProsima/Fast-CDR` @ `2.2.x`,
   `eProsima/foonathan_memory_vendor`, `ros2/rmw_fastrtps` @ `jazzy` — the exact set `ros2.repos`
   lists) — rebuild these in a colcon workspace overlay with that cmake arg, then source the
   overlay *before* the rest of the workspace so it shadows the apt-installed Fast DDS.
2. `fastdds_statistics_backend` built and installed against that same Fast-DDS. **Pin the `v1.1.0`
   tag** — `main` and every tagged release from `v2.0.0` onward require Fast-DDS **3.0.0**
   (`find_package(fastdds 3.0.0 REQUIRED)`), which Jazzy doesn't ship; only `v1.0.0`/`v1.1.0`
   target Fast-DDS `>=2.13.0`. No rosdep/apt key exists for any distro, so `rosdep install` never
   pulls it in and `dc_measurements/package.xml` deliberately does not list it as a `<depend>`.
3. The `FASTDDS_STATISTICS` environment variable, set on every process before it creates its
   first DomainParticipant — the library only *emits* the DataKinds named in it (semicolon-
   separated topic aliases), regardless of whether the plugin is built and running:

   ```bash
   export FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;SUBSCRIPTION_THROUGHPUT_TOPIC;RTPS_SENT_TOPIC;RTPS_LOST_TOPIC"
   ```

   Without it, `participant_count`/`datawriter_count`/`datareader_count`/`participants`/`hosts`/
   `users`/`process_names` still populate (basic discovery data), but `latency_ns_mean` and every
   throughput/RTPS-packet field stay permanently absent — not intermittently, every single poll.

Without #1/#2, `find_package(fastdds_statistics_backend)` fails at CMake configure time and the
plugin — and its test — are skipped from the build entirely.

## Parameters

| Parameter     | Description                                                    | Type | Default   |
| ------------- | ---------------------------------------------------------------- | ---- | --------- |
| **domain_id** | DDS domain to monitor (the same value `ROS_DOMAIN_ID` would use) | int  | 0 (Optional) |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "FastddsStats",
  "properties": {
    "event": { "type": "string", "enum": ["sample"] },
    "domain_id": { "type": "integer", "minimum": 0 },
    "participant_count": { "type": "integer", "minimum": 0 },
    "datawriter_count": { "type": "integer", "minimum": 0 },
    "datareader_count": { "type": "integer", "minimum": 0 },
    "latency_ns_mean": { "type": "number", "minimum": 0 },
    "publication_throughput_bytes_per_sec_mean": { "type": "number", "minimum": 0 },
    "subscription_throughput_bytes_per_sec_mean": { "type": "number", "minimum": 0 },
    "rtps_packets_sent": { "type": "integer", "minimum": 0 },
    "rtps_packets_lost": { "type": "integer", "minimum": 0 },
    "participants": { "type": "array" },
    "hosts": { "type": "array", "items": { "type": "string" } },
    "users": { "type": "array", "items": { "type": "string" } },
    "process_names": { "type": "array", "items": { "type": "string" } }
  },
  "required": ["event", "domain_id", "participant_count", "datawriter_count", "datareader_count"],
  "type": "object"
}
```

The full file (`plugins/measurements/json/fastdds_stats.json`) also spells out each
`participants[]` entry's `name`/`guid` properties.

## Measurement configuration

```yaml
...
fastdds_stats:
  plugin: "dc_measurements/FastddsStats"
  topic_output: "/dc/measurement/fastdds_stats"
  polling_interval: 5000
  domain_id: 0
```

Example Record data, captured from a real run (all three prerequisites above met):

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

`latency_ns_mean` and every throughput/RTPS field are absent here — genuinely, not a
capture artifact: nothing exchanged data on a matched DataWriter/DataReader pair
within this particular 5-second poll window, and per the Statistics Backend's own
contract, absence is how "nothing to report" is signaled, not a zero. A busier DDS
graph (more topics, higher rate) makes them appear more often, not guaranteed every
poll.
```
