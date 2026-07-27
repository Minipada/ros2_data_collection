# Data Pipeline

This page follows one piece of data from the sensor that produced it to the external
system that stores it. The vocabulary — Measurement, Record, Group, File, Bridge,
Shipper, Destination, Tag — is defined in [Concepts](./concepts.md) and used
consistently throughout.

## The path of a Record

```mermaid
flowchart LR
    subgraph ros["ROS 2 graph"]
        meas["Measurement plugins<br/>(measurement_server)"]
        cond["Conditions"]
        group["Group node<br/>(group_server)"]
    end
    subgraph bridge["Bridge (dc_bridge)"]
        fwd["Forwarder"]
        upl["Uploader"]
    end
    subgraph shipper["Shipper (Vector)"]
        route["dc.&lt;tag&gt; routes"]
        buf[("Disk buffer")]
    end
    subgraph dest["Destinations"]
        pg["PostgreSQL"]
        s3["S3-compatible storage"]
        other["Any Vector sink<br/>(passthrough)"]
    end

    cond -- gate --> meas
    meas -- "Records (StringStamped)" --> fwd
    meas -- "Records" --> group
    group -- "merged Records" --> fwd
    meas -. "Files on disk" .-> upl
    fwd -- "shipper ingest protocol" --> route
    route --> buf
    buf --> pg
    buf --> other
    upl -- "File bytes" --> s3
    upl -- "status Records (dc.files)" --> fwd
```

1. **A Measurement produces a Record.** Each Measurement plugin samples its source on a
   timer (or on an input topic) and publishes one timestamped JSON document as a
   `dc_interfaces/msg/StringStamped` on its `topic_output`. **Conditions** can gate
   whether the Measurement collects at all.
2. **Optionally, a Group merges Records.** The Group node subscribes to several
   Measurement topics and publishes one merged Record on `/dc/group/<name>` once their
   timestamps line up (`sync_delay`).
3. **The Bridge forwards every Record.** `dc_bridge` subscribes to every topic listed in
   any Destination's `inputs`, derives that topic's **Tag**, and hands the Record to the
   Shipper over the local shipper ingest socket (default `127.0.0.1:24224`), with
   receipt acknowledgement.
4. **The Shipper routes, buffers and delivers.** Vector normalizes the Record's
   timestamp field, exposes it on the public `dc.<tag>` route, writes it to a persistent
   disk buffer, and delivers it to each Destination wired to that route — retrying with
   its own backoff until it succeeds.
5. **Files take a different path.** A **File** (camera image, map, video) is never sent
   through the Shipper. The Bridge's **Uploader** reads the `local_paths` /
   `remote_paths` references embedded in the Record, uploads the bytes directly to
   object storage, verifies them, and emits a *status Record* under the `dc.files` Tag —
   which then travels the ordinary Record path. See
   [File uploads](./destinations.md#file-uploads-receives-files-the-uploader-adr-0005).

## Where each piece is configured

| Stage                          | Node               | Parameters                                     |
| ------------------------------ | ------------------ | ---------------------------------------------- |
| Producing Records              | `measurement_server` | [Measurements](./measurements.md)             |
| Gating collection              | `measurement_server` | [Conditions](./conditions.md)                 |
| Merging Records                | `group_server`     | [Groups](./groups.md)                          |
| Routing, buffering, delivering | `dc_bridge`        | [Destinations](./destinations.md)              |

Routing is decided in exactly one place: a Destination's `inputs` list names the topics
it receives. Nothing on the producing side selects a Destination.

## Deterministic startup ordering

`dc_bringup.launch.py` brings the pipeline up in a fixed order (ADR-0006), so no Record
can be emitted before the pipeline is able to accept it:

1. **Bridge first.** `dc_bridge` starts as a plain node (outside the lifecycle manager)
   and spawns the Vector Shipper as a supervised child process. The measurement server
   also starts here, but stays unconfigured and inactive — its publishers cannot emit
   anything yet.
2. **Readiness gate.** A `bridge_ready_gate` process blocks, polling the Bridge's
   `~/ready` service (`std_srvs/Trigger`), which answers `success=True` only once the
   Shipper is accepting connections on its ingest socket. The gate's `service`,
   `timeout_s` (default 120 s), and `poll_interval_s` parameters are configurable from
   the params file under `bridge_ready_gate:`.
3. **Activation.** Only when the gate exits successfully does the launch start
   `lifecycle_manager_dc`, which configures and then activates the collection nodes. If
   the Bridge never becomes ready before the gate's deadline, the whole launch shuts
   down loudly instead of leaving a half-started pipeline running.

## Durability and supervision

- **Disk buffering.** The Shipper owns a persistent disk buffer at `shipper.data_dir`.
  A Record the Bridge has handed over survives a Destination outage, a Bridge restart,
  and a robot reboot; delivery resumes — with end-to-end acknowledgements — once the
  Destination is reachable again.
- **Backpressure.** When a Destination is slow, the Shipper stops acknowledging, and the
  Bridge propagates that backwards rather than dropping data silently.
- **Supervision.** The launch file respawns `dc_bridge` unconditionally (independent of
  `use_respawn`), and the Bridge supervises its Shipper child — including a Linux
  parent-death signal, so the Shipper can never outlive the Bridge even across a
  SIGKILL or crash.
- **The one lossy window.** Records published while the Bridge is down are dropped: ROS
  topics are fire-and-forget and nothing buffers upstream of the Bridge. Delivery
  resumes as soon as the respawned Bridge is ready.
- **Delivery semantics.** At-least-once. After a crash or an induced outage, a boundary
  Record may be re-sent; deduplicate on read if that matters to you.
