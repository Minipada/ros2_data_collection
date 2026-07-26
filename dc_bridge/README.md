# dc_bridge

The Bridge (ADRs 0001/0003/0005/0006; ADR-0007 reverts the ADR-0004 Rust pilot): a C++
(`rclcpp`) node that subscribes to `dc_interfaces/msg/StringStamped` Record topics and
forwards every Record to the [Vector](https://vector.dev) shipper over the Fluent Forward
protocol. It renders Vector's configuration from ROS parameters for the blessed
Destination set (`postgres`, `s3`, `file`, `console` — ADR-0003) and passes raw Vector
snippets (`custom_config_files`) through for everything else. It also hosts the
**Uploader** (ADR-0005): `receives: files` Destinations are served by the Bridge itself —
Records referencing Files get their Files uploaded to S3-compatible object storage
(multipart + resumable), verified, and reported as status/metadata Records under the
`dc.files` Tag.

`dc_bridge` is an ordinary `ament_cmake` C++ package. It builds with the same `rosdep
install` + `colcon build` as every other `dc_*` package — no Rust toolchain, no
`colcon-cargo`, no source-built message repos. See `docs/adr/0007-bridge-returns-to-cpp.md`
for why the Rust pilot was reverted.

## Layout

- **`include/dc_bridge/` + `src/`** — the ROS-independent core, built as `dc_bridge_core`
  and unit-tested with plain gtest (no ROS or cloud needed):
  - `forwarder` — Fluent Forward msgpack framing (msgpack-cxx), socket lifecycle, lazy
    reconnection, and backpressure, behind one `send(record)` call.
  - `supervisor` — spawns and restarts the vendored Vector binary; sets
    `PR_SET_PDEATHSIG(SIGKILL)` so Vector can never outlive the Bridge, even on a
    SIGKILL/crash.
  - `readiness` — a TCP-connect probe against Vector's Forward port + a shared ready flag.
  - `topic_config` — derives a Fluent Forward tag from a ROS topic name.
  - `render` — the ADR-0003 config renderer: ROS parameters → a complete Vector TOML
    config (toml++). Pure, gold-file tested.
  - `vector_binary` — locates the vendored Vector binary on `AMENT_PREFIX_PATH`.
  - `uploader/` — the Uploader (ADR-0005): `group` (parses the Files a Record
    references), `content_type` (magic-byte sniffing), `status` (the Humble-compatible
    status-row shapes), `multipart` (resumable multipart with a per-part JSON checkpoint
    sidecar), and `uploader` (the verify-then-delete orchestration). All built on an
    abstract `ObjectStore` interface, so the logic is tested against an in-memory fake
    with no cloud dependency.
- **`src/uploader/s3_object_store.cpp`** — the aws-sdk-cpp implementation of
  `ObjectStore` (the only Uploader piece that links the SDK, via `aws_sdk_vendor`).
  Verified against RustFS (PutObject + multipart) before adoption.
- **`src/bridge_node.cpp` / `src/main.cpp`** — the `rclcpp` node: declares the
  `shipper`/`destinations`/`files` parameters (ADR-0003 config contract + ADR-0005),
  renders and `vector validate`s the config, spawns/supervises Vector, subscribes to
  every Destination's `inputs` topics, forwards Records, runs the Uploader worker thread,
  and exposes a `~/ready` (`std_srvs/Trigger`) service. `rclcpp` handles SIGINT/SIGTERM
  and `on_shutdown` stops Vector, so it's never orphaned.
- **`test/`** — gtest suites (`forwarder`, `supervisor`, `render`, `misc`, `uploader`),
  all linking only the aws-free core.

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

It renders: the Fluent Forward source; one `remap` (VRL) transform normalizing each
destination's timestamp into its `time_key`; one `route` transform whose branches expose
every distinct Tag at the public `dc.<tag>` output (ADR-0003's passthrough contract, see
`doc/src/dc/destinations.md`); a disk buffer per persistent sink; and the blessed sinks.
`validate_custom_config_files` collision-checks passthrough snippets, and `main.cpp` runs
`vector validate --no-environment` over the merged config before starting Vector, so a
bad config fails loudly at startup rather than crash-looping.

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
renderer / Uploader logic:

```bash
cmake -S dc_bridge/test/local -B build && cmake --build build && ctest --test-dir build
```

The store-backed end-to-end verification (a Record's File landing in a real S3 bucket,
status rows in Postgres) lives in the zero-loss E2E harness — see `tools/e2e/README.md`.
