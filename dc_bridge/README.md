# dc_bridge

The Bridge (ADRs 0001/0003/0005/0006/0007): a C++ (`rclcpp`) node that subscribes to
`dc_interfaces/msg/StringStamped` Record topics and
forwards every Record to the [Vector](https://vector.dev) shipper over the shipper
ingest protocol (the open Fluentd "Forward" wire format — no Fluentd/Fluent Bit software
runs anywhere in the pipeline; see CONTEXT.md). Delivery is confirmed end-to-end (#266):
every frame carries a `chunk` id and Vector's `ack` response for it clears that Record
from an in-memory unacked window, so a Record is only forgotten once the Shipper has
actually durably buffered it — see "Delivery guarantees" below. It renders Vector's
configuration from ROS parameters for the blessed Destination set (`postgres`, `s3`,
`file`, `console` — ADR-0003) and passes raw Vector snippets (`custom_config_files`)
through for everything else. It also hosts the
**Uploader** (ADR-0005): `receives: files` Destinations are served by the Bridge itself —
Records referencing Files get their Files uploaded to S3-compatible object storage
(multipart + resumable), verified, and reported as status/metadata Records under the
`dc.files` Tag. Pending uploads are durable (#265): each is written to a disk-backed
intent queue before being handed to the Uploader, so a Bridge restart replays whatever
never got acked instead of forgetting it. That queue and the Uploader's multipart-resume
state live under `uploader.data_dir` (#441), a directory separate from the Shipper's own
`shipper.data_dir` buffer — they default to the same path, so a deployment that only ever
set `shipper.data_dir` keeps working unchanged, but each can be mounted as its own volume.

`dc_bridge` is an ordinary `ament_cmake` C++ package. It builds with the same `rosdep
install` + `colcon build` as every other `dc_*` package. See
`docs/adr/0007-bridge-returns-to-cpp.md` for the architecture decision history.

## Layout

- **`include/dc_bridge/` + `src/`** — the ROS-independent core, built as `dc_bridge_core`
  and unit-tested with plain gtest (no ROS or cloud needed):
  - `forwarder` — shipper ingest protocol msgpack framing (msgpack-cxx), socket
    lifecycle, lazy reconnection, backpressure, and confirmed delivery (#266: `chunk`/
    `ack`, an in-memory unacked window, resend on reconnect/ack-timeout), behind one
    `send(record)` call plus a `poll()` for idle draining.
  - `supervisor` — spawns and restarts the vendored Vector binary; sets
    `PR_SET_PDEATHSIG(SIGKILL)` so Vector can never outlive the Bridge, even on a
    SIGKILL/crash.
  - `readiness` — a TCP-connect probe against Vector's ingest port + a shared ready flag.
  - `topic_config` — derives a shipper ingest protocol Tag from a ROS topic name.
  - `raw_config` — raw-mode (#227) policy: the include/exclude filter, the
    `dc.raw.<topic>` Tag derivation, the per-topic rate limiter and the drop counters.
  - `render` — the ADR-0003 config renderer: ROS parameters → a complete Vector TOML
    config (toml++). Pure, gold-file tested.
  - `vector_binary` — locates the vendored Vector binary on `AMENT_PREFIX_PATH`.
  - `atomic_write` — writes a file crash/partial-write-safe (write to `<path>.tmp`, then
    `rename()`, #444), used for the rendered Shipper config so a reader polling the path
    never observes a partial write.
  - `uploader/` — the Uploader (ADR-0005): `group` (parses the Files a Record
    references), `content_type` (magic-byte sniffing), `status` (the Humble-compatible
    status-row shapes), `multipart` (resumable multipart with a per-part JSON checkpoint
    sidecar), `intent_queue` (#265 — the disk-backed durable queue of pending uploads:
    crash-atomic tmp+rename enqueue, ack-by-unlink, oldest-first sweep with per-entry
    exponential backoff, replayed on startup), and `uploader` (the verify-then-delete
    orchestration). All built on an abstract `ObjectStore` interface, so the logic is
    tested against an in-memory fake with no cloud dependency.
- **`src/raw_subscriptions.cpp`** (`dc_bridge_ros`) — raw / generic-subscription mode's
  rclcpp half (#227): topic discovery, `create_generic_subscription`, and the runtime
  introspection walk that turns a message of a type the Bridge was never compiled against
  into JSON. Its own library rather than a file in the executable, so the conversion is
  gtested against real message types without the node, Vector or the AWS SDK.
- **`src/uploader/s3_object_store.cpp`** — the aws-sdk-cpp implementation of
  `ObjectStore` (the only Uploader piece that links the SDK, via `aws_sdk_vendor`).
  Verified against RustFS (PutObject + multipart) before adoption.
- **`src/bridge_node.cpp` / `src/main.cpp`** — the `rclcpp` node: declares the
  `shipper`/`uploader`/`destinations`/`files` parameters (ADR-0003 config contract + ADR-0005),
  renders and atomically writes the config (write then rename, #444), subscribes to every
  Destination's `inputs` topics, forwards Records, runs the Uploader worker thread, and
  exposes a `~/ready` (`std_srvs/Trigger`) service. In the default **managed** mode
  (`shipper.managed: true`) it also locates the vendored Vector binary, `vector
  validate`s the merged config, and spawns/supervises Vector. In **unmanaged** mode
  (`shipper.managed: false`, #440/#444 — the split-deployment topology) it locates no
  binary, spawns no child, and installs no parent-death signal: an orchestrator owns the
  Shipper container/pod instead, and the Bridge only renders the config (to
  `shipper.config_path`, configurable so it can land on a volume shared with that
  container) and connects. `~/ready` reports ready the same way in both modes — a TCP
  probe against the Shipper's ingest port, independent of who spawned it. `rclcpp` handles
  SIGINT/SIGTERM and `on_shutdown` stops Vector in managed mode (a no-op in unmanaged
  mode, nothing to stop), so it's never orphaned.
- **`test/`** — gtest suites (`forwarder`, `supervisor`, `render`, `misc`, `uploader`,
  `intent_queue`), all linking only the aws-free core.

## Config renderer (ADR-0003)

`dc_bridge::render` maps the unified config contract to a complete Vector configuration:

```yaml
dc_bridge:
  ros__parameters:
    shipper:
      data_dir: "$HOME/.dc/buffer"
    destinations: ["pgsql"]
    pgsql:
      type: postgres            # blessed types: postgres | s3 | file | console
      receives: records          # records (default) | files
      inputs: ["/dc/group/robot"]
      host: "127.0.0.1"
      port: 5432
      user: "dc"
      password: "$DC_PG_PASSWORD"   # $VAR / ${VAR} env expansion
      database: "dc"
      table: "dc"
      time_key: "date"
      time_format: "double"      # double | iso8601
```

It renders: the shipper ingest protocol source (with global acknowledgements enabled,
#266); one `remap` (VRL) transform normalizing each destination's timestamp into its
`time_key`; one `route` transform whose branches expose every distinct Tag at the public
`dc.<tag>` output (ADR-0003's passthrough contract, see `doc/src/dc/destinations.md`); a
disk buffer per persistent sink; and the blessed sinks.
`validate_custom_config_files` collision-checks passthrough snippets, and `main.cpp` runs
`vector validate --no-environment` over the merged config before starting Vector, so a
bad config fails loudly at startup rather than crash-looping.

## Raw / generic-subscription mode (#227)

Beside the Record topics above, the Bridge can subscribe to topics it was **never
compiled against** — `rclcpp::GenericSubscription` plus the runtime introspection type
support, the same mechanism `ros2 bag record -a` uses — and ship every message as a
Record under the `dc.raw.<topic>` Tag:

```yaml
    raw:
      enabled: true
      destination: "raw_console"   # a configured `receives: records` Destination
      include: ["^/"]              # topic-name regexes; see doc/src/dc/raw_topics.md
      max_rate_hz: 10.0            # per topic; raw mode sheds at the source
```

Two things about this are worth knowing at the code level:

- **The whole Tag namespace shares one Vector route.** Topics are discovered while Vector
  is already running, and a rendered config can't grow exact-Tag branches without a
  restart — so `Destination::tag_prefixes` renders a `starts_with(.tag, "dc.raw.")`
  branch at `dc.dc.raw` (`route_output_for_tag_prefix`). It is the only routing
  construct in the renderer that matches something other than an exact Tag.
- **Raw Records are dropped, never queued.** A raw Record the Forwarder refuses
  (backpressure, dropped connection) is counted and discarded rather than kept in the
  unacked window: a firehose topic would otherwise fill that window and push real
  Measurement Records out of it. The counters are on `~/ready`.

`ros2 launch dc_bringup dc_raw.launch.py` runs the Bridge in this mode with no collection
nodes at all. See [doc/src/dc/raw_topics.md](../doc/src/dc/raw_topics.md) for the
configuration reference and the volume/backpressure contract.

## Delivery guarantees (#266)

The Bridge→Shipper hop is **at-least-once**, not exactly-once. Every frame the Forwarder
sends carries a unique `chunk` id (the shipper ingest protocol's built-in
acknowledgement option); Vector's `fluent` source is configured with global
`acknowledgements.enabled = true` and replies with an `ack` for each chunk once the
Record has been durably buffered (confirmed against the real pinned Vector binary with
its sink completely unreachable: the `ack` still comes back immediately, because
acknowledgement happens at the disk buffer, not at final delivery to the Destination).
The Forwarder keeps every sent-but-unacked Record in an in-memory window and resends it:

- unconditionally, after any reconnect (the peer may never have seen it), and
- after `ack_timeout` (default 5s) with no response (the ack itself may have been lost).

Duplicates can result from either path — a Record acked right as the Forwarder decided to
resend it lands twice — so consumers that care about exact counts (dashboards, joins)
should dedupe downstream on their own key, same as Humble/Fluent Bit's chunk-retry
semantics. What this **does** guarantee: a Record is never silently forgotten just
because Vector was mid-restart or the TCP link blipped.

Two independent bounds protect the Bridge itself:

- **The unacked window** (`ForwarderConfig::max_unacked_records` /
  `max_unacked_bytes`, default 10 000 records / 16 MiB): if Vector is unreachable *and*
  its own disk buffer is also full for long enough that acks stop arriving at all, the
  window drops the oldest unacked record to make room, with a rate-limited
  `RCLCPP_WARN`. Reaching this means the outage has outlasted what
  `shipper.buffer_max_bytes` (below) can absorb — the window is a bounded safety net for
  *in-flight* data, not a substitute for sizing the real buffer.
- **Vector's own disk buffer** (`shipper.buffer_max_bytes`, ADR-0002): sized for how long
  a Destination can be down before the Bridge's window above starts shedding data. A
  Bridge crash *concurrent with* a Vector/Destination outage still loses the in-memory
  window (out of scope here, tracked as a known double-failure — see ADR-0002 and #265's
  durable Uploader intent queue for how the File-upload side handles the analogous case).

## Dependencies

All `rosdep`-resolvable, so `rosdep install --from-paths src --ignore-src` covers them:
`rclcpp`, `dc_interfaces`, `std_msgs`, `std_srvs`, `nlohmann-json-dev`, and `aws_sdk_vendor`
(builds the AWS SDK for C++ from source — the Uploader's S3 client). Two header-only C++
libraries — `tomlplusplus` and `msgpack-cxx` — have no upstream rosdistro key, so this
repo ships a small local rosdep source (`rosdep/dc.yaml`). Register it once before
`rosdep install`:

```bash
echo "yaml file://$(pwd)/rosdep/dc.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/10-dc.list
rosdep update
```

(CI does this automatically in `tools/e2e/Containerfile`.)

## Building and testing

Inside a colcon workspace with this repo under `src/`:

```bash
colcon build --packages-up-to dc_bridge
colcon test --packages-select dc_bridge
```

The ROS-independent core also builds and tests without a ROS install, via a standalone
CMake project (`test/local/`) — handy for fast iteration on the Forwarder / Supervisor /
renderer / Uploader / intent queue logic:

```bash
cmake -S dc_bridge/test/local -B build && cmake --build build && ctest --test-dir build
```

The store-backed end-to-end verification (a Record's File landing in a real S3 bucket,
status rows in Postgres) lives in the zero-loss E2E harness — see `tools/e2e/README.md`.
