# Data Pipeline

This page follows one piece of data from the sensor that produced it to the external
system that stores it. The vocabulary — Measurement, Record, Group, File, Bridge,
Shipper, Destination, Tag — is defined in [Concepts](./concepts.md) and used
consistently throughout.

## C4 model

Three levels, each zooming further into the pipeline: DC as a single system among
external actors and destinations (Context), the processes that make it up (Container),
and the pieces inside the Bridge (Component). The flowchart in
[the next section](#the-path-of-a-record) stays as the at-a-glance narrative view of a
single Record's journey; these diagrams complement it rather than replace it.

### Context (C1)

DC runs as one system on the robot. A robot operator configures it; analytics and
dashboard consumers read from whatever Destinations it was configured to write to —
PostgreSQL, S3-compatible object storage, and (via passthrough, ADR-0003) any other
Shipper-supported sink.

```mermaid
C4Context
    title System context for DC (Data Collection)

    Person(operator, "Robot Operator", "Configures Measurements, Conditions, Groups, and Destinations via ROS params")
    Person(consumer, "Analytics / Dashboard Consumer", "Queries Destinations for fleet dashboards and alerting")

    System(dc, "DC (Data Collection)", "Collects operational data from the robot's ROS 2 graph, validates it, and routes it to external infrastructure")

    SystemDb_Ext(postgres, "PostgreSQL", "Blessed Destination for structured Records")
    System_Ext(objectStorage, "S3-compatible object storage", "RustFS / MinIO / S3 — blessed Destination for Files, and for Records")
    System_Ext(otherSinks, "Other passthrough sinks", "Any Shipper-supported destination not natively blessed by DC (Kafka, InfluxDB, Slack, HTTP, ...)")

    Rel(operator, dc, "Configures")
    Rel(dc, postgres, "Delivers Records")
    Rel(dc, objectStorage, "Uploads Files, delivers Records")
    Rel(dc, otherSinks, "Delivers Records (passthrough, ADR-0003)")
    Rel(consumer, postgres, "Queries")
    Rel(consumer, objectStorage, "Retrieves Files")
```

### Container (C2)

Inside DC, only three nodes are lifecycle-managed by `dc_lifecycle_manager`:
`measurement_server` and `group_server`. The Bridge (`dc_bridge`) and its supervised
Shipper child are deliberately outside that boundary (ADR-0006) — the Bridge has no
meaningful deactivated state, so its readiness comes from launch ordering
(`bridge_ready_gate`) instead of a lifecycle transition. See
[Deterministic startup ordering](#deterministic-startup-ordering) for the sequence this
diagram's `bridge_ready_gate` → `dc_lifecycle_manager` relationship summarizes.

```mermaid
C4Container
    title Container diagram for DC (Data Collection)

    Person(operator, "Robot Operator")

    System_Boundary(dc, "DC (Data Collection)") {
        Boundary(lifecycle, "Lifecycle-managed by dc_lifecycle_manager (ADR-0006)") {
            Container(measurementServer, "measurement_server", "C++ / rclcpp lifecycle node", "Hosts Measurement and Condition plugins; publishes Records")
            Container(groupServer, "group_server", "C++ / rclcpp lifecycle node", "Merges Records from several Measurements by time proximity")
        }
        Container(lifecycleManager, "dc_lifecycle_manager", "C++ / rclcpp, nav2-style", "Configures then activates the lifecycle-managed nodes once the readiness gate passes")
        Container(gate, "bridge_ready_gate", "Python / rclpy, launch-time process", "Blocks launch until dc_bridge's ~/ready service reports success")
        Container(bridge, "dc_bridge", "C++ / rclcpp, plain node outside lifecycle (ADR-0006, ADR-0007)", "Forwards Records to the Shipper, uploads Files, renders Shipper config, owns readiness")
        Container(shipper, "Shipper (Vector)", "external process, supervised child of dc_bridge (ADR-0001, ADR-0002)", "Routes, buffers, and retries delivery to Destinations")
    }

    SystemDb_Ext(postgres, "PostgreSQL")
    System_Ext(objectStorage, "S3-compatible object storage")
    System_Ext(otherSinks, "Other passthrough sinks")

    Rel(operator, lifecycleManager, "ros2 launch dc_bringup")
    Rel(bridge, shipper, "Spawns and supervises (fork/exec/waitpid, PR_SET_PDEATHSIG)")
    Rel(gate, bridge, "Polls ~/ready")
    Rel(gate, lifecycleManager, "Unblocks launch once ready (OnProcessExit)")
    Rel(lifecycleManager, measurementServer, "Configure, then activate")
    Rel(lifecycleManager, groupServer, "Configure, then activate")
    Rel(measurementServer, bridge, "Records (StringStamped topics)")
    Rel(measurementServer, groupServer, "Records (StringStamped topics)")
    Rel(groupServer, bridge, "Merged Records (StringStamped topics)")
    Rel(measurementServer, bridge, "Files on disk (local_paths / remote_paths)")
    Rel(bridge, shipper, "Records over the shipper ingest protocol (msgpack, :24224)")
    Rel(shipper, postgres, "Delivers Records")
    Rel(shipper, otherSinks, "Delivers Records (passthrough)")
    Rel(bridge, objectStorage, "Uploads Files, delivers Records")
```

### Component (C3) — the Bridge

The pieces added across #244–#267, now invisible from the outside: `BridgeNode` wires a
Forwarder (Records → Shipper), a Supervisor (owns the Vector child process), a Config
renderer (ADR-0003's `shipper`/`destinations` params → Vector TOML, including
passthrough snippet validation), Readiness (backs `~/ready`), and — for
`receives: files` Destinations (ADR-0005) — a durable IntentQueue feeding the Uploader,
which verifies File uploads against an S3-compatible ObjectStore and reports status
Records back through the same Forwarder under the `dc.files` Tag.

```mermaid
C4Component
    title Component diagram for dc_bridge (the Bridge)

    Container(measurementServer, "measurement_server", "external container")
    Container(shipper, "Shipper (Vector)", "external process")
    System_Ext(objectStorage, "S3-compatible object storage")

    Container_Boundary(bridge, "dc_bridge") {
        Component(node, "BridgeNode", "rclcpp::Node", "Wires the components together; owns the ~/ready service and topic subscriptions")
        Component(renderer, "Config renderer", "C++ (render.*)", "Renders shipper/destinations ROS params into Vector TOML (ADR-0003); validates custom_config_files passthrough snippets")
        Component(supervisor, "Supervisor", "C++", "fork/exec/waitpid + PR_SET_PDEATHSIG; restarts Vector on crash with backoff")
        Component(readiness, "Readiness", "C++", "Shared ready flag plus a TCP probe against Vector's ingest port; backs the ~/ready service")
        Component(forwarder, "Forwarder", "C++", "Sends Records to Vector over the shipper ingest protocol (msgpack), with reconnection and backpressure")
        Component(topicConfig, "TopicConfig", "C++", "Derives a Tag from a ROS topic name")
        Component(uploader, "Uploader", "C++ (ADR-0005)", "Per File x storage verify-or-upload, group-completion marker, delete_when_sent")
        Component(intentQueue, "IntentQueue", "C++ (#265)", "Disk-backed durable FIFO of pending upload intents; crash-atomic enqueue/ack, oldest-first with per-entry backoff")
        Component(objectStoreClient, "ObjectStore client", "C++, AWS SDK for C++", "Uploads and verifies File bytes on S3-compatible storage")
        Component(retention, "Retention", "C++ (#267)", "Sheds the oldest unverified intents under disk pressure; a no-op sweep when disabled")
    }

    Rel(measurementServer, node, "Records (StringStamped topic subscriptions)")
    Rel(node, renderer, "Renders Vector config at startup")
    Rel(node, supervisor, "Starts and supervises")
    Rel(supervisor, shipper, "spawn / waitpid / restart")
    Rel(node, readiness, "Reads and writes the ready flag")
    Rel(readiness, shipper, "TCP probe of the ingest port")
    Rel(node, forwarder, "Records from Records-Destination topics")
    Rel(forwarder, topicConfig, "Derives each topic's Tag")
    Rel(forwarder, shipper, "shipper ingest protocol")
    Rel(node, intentQueue, "Enqueues Records from Files-Destination topics")
    Rel(intentQueue, uploader, "Replays the oldest ready intent")
    Rel(uploader, objectStoreClient, "Upload / verify")
    Rel(objectStoreClient, objectStorage, "PutObject, multipart, HEAD")
    Rel(uploader, forwarder, "Status Records under dc.files")
    Rel(retention, intentQueue, "Sheds oldest unverified intents under disk pressure")
```

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
