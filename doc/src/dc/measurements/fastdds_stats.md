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
  OS users and processes running them (`hosts`, `users`, `processes`) — useful for spotting which
  machine or process is actually behind a noisy participant on a multi-process robot

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
   default, and most distro/apt Fast DDS builds do not enable it.
2. `fastdds_statistics_backend` built and installed (from source; see
   [the eProsima docs](https://fast-dds-statistics-backend.readthedocs.io) — it has no rosdep/apt
   key for any distro at the time of writing, so `rosdep install` never pulls it in and
   `dc_measurements/package.xml` deliberately does not list it as a `<depend>`).

Without both, `find_package(fastdds_statistics_backend)` fails at CMake configure time and the
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
    "processes": { "type": "array", "items": { "type": "string" } }
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

Example Record data:

```json
{
  "event": "sample",
  "domain_id": 0,
  "participant_count": 4,
  "datawriter_count": 6,
  "datareader_count": 7,
  "latency_ns_mean": 182345.2,
  "publication_throughput_bytes_per_sec_mean": 10432.0,
  "subscription_throughput_bytes_per_sec_mean": 9880.5,
  "rtps_packets_sent": 214,
  "rtps_packets_lost": 0,
  "participants": [
    { "name": "measurement_server", "guid": "01.0f.2a.3c.00.00.00.00.00.00.00.00|0.0.1.c1" }
  ],
  "hosts": ["robot-01"],
  "users": ["dc"],
  "processes": ["measurement_server-12345"]
}
```
