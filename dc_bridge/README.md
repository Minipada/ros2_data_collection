# dc_bridge

The Bridge (ADRs 0001/0004/0006): a Rust (`rclrs`) node that subscribes to `dc_interfaces/msg/StringStamped`
Record topics and forwards every Record to the Vector shipper over the Fluent Forward
protocol. This package is the DC 2.0 tracer bullet proving Measurement → Bridge →
Shipper → output end-to-end, with a console sink standing in for real Destinations.

## Layout

- **`dc_bridge_core/`** — the ROS-independent core: `Forwarder` (msgpack framing, socket
  lifecycle, reconnection, backpressure), `Supervisor` (generic process respawn),
  `Readiness` (TCP-accept probe), `config` (topic → Fluent Forward tag), and
  `vector_config` (renders the minimal Vector config, locates the vendored binary). It's
  a standalone crate with its own `Cargo.toml` (declaring `[workspace]` so it never gets
  swept into the outer package's workspace) and zero ROS dependencies, so it builds and
  tests with plain Cargo:

  ```sh
  cd dc_bridge/dc_bridge_core
  cargo test
  ```

  This is how the Forwarder/Supervisor/Readiness acceptance criteria for this package
  were verified in a sandbox with no ROS 2 or ros2_rust install available — no `colcon`
  or `rclrs` toolchain is required to run these tests.

- **`src/main.rs`** — the actual ROS node. It declares parameters (`topics`,
  `vector_forward_host`, `vector_forward_port`, `vector_binary`), starts the Supervisor
  against the vendored `vector_vendor` binary, subscribes to each configured topic, and
  exposes a `~/ready` (`std_srvs/Trigger`) readiness service. It depends on `rclrs`,
  `dc_interfaces`, `std_msgs`, and `std_srvs`, all only resolvable inside a colcon
  workspace (see below).

## Verified in a real ROS 2 Jazzy + ros2_rust environment

`colcon build --packages-select dc_bridge` succeeds, and the full pipeline was run
end-to-end in a Docker container (`ros:jazzy-ros-base` + rustup + `colcon-cargo`/
`colcon-ros-cargo` + the Jazzy message repos + `rosidl_rust`, per the recipe below):
`ros2 run dc_bridge dc_bridge` locates and spawns the vendored Vector binary, the
`~/ready` service answers `true` once Vector is listening, and a `StringStamped`
published on a subscribed topic shows up on Vector's console sink with the derived tag,
e.g. publishing `{data: '{"uptime_s": 42}'}` on `/dc/measurement/uptime` produced:

```
{"host":"127.0.0.1","source_type":"fluent","tag":"dc.measurement.uptime","timestamp":"1970-01-01T00:00:00Z","uptime_s":42}
```

Killing the real (vendored) Vector process was also confirmed to trigger an automatic
respawn (a new PID, Vector re-listening) via the Supervisor, and readiness recovers
to `true` afterward.

Two real bugs surfaced only in this environment (unit tests can't catch either one,
since they're specific to the generated/real rclrs API surface):

- **`rosidl_runtime_rs` version skew**: the `rosidl_rust` code generator's git `main`
  branch (the version the Jazzy build recipe below tells you to clone) emits idiomatic
  message conversions (`Sequence<T> -> Vec<T>` for unbounded primitive-array fields,
  e.g. `dc_interfaces`' `DrawImage.srv` `uint16[] color`) that require a
  `rosidl_runtime_rs` trait impl not yet in the crates.io `0.6.0` release. Fixed with a
  `[patch.crates-io]` in `Cargo.toml` pointing `rosidl_runtime_rs` at its own git `main`
  branch, matching how the generator itself is sourced.
- **Missing trait imports**: `Context::create_basic_executor` and
  `Vec<RclrsError>::first_error` are extension-trait methods (`CreateBasicExecutor`,
  `RclrsErrorFilter`) that the ros2_rust examples get "for free" via `use rclrs::*`;
  explicit imports were needed here.

## Building via colcon (cargo integration)

Unlike the C++ packages here, `dc_bridge` uses `ament_cargo` as its `package.xml` build
type instead of `ament_cmake`. colcon does not understand Cargo natively; building Rust
ROS packages requires the [ros2_rust](https://github.com/ros2-rust/ros2_rust) tooling.
This is the recipe used to verify this package (see above):

1. Install Rust (`rustup`) and `libclang-dev` (for bindgen), then
   `pip install --break-system-packages colcon-cargo colcon-ros-cargo` — these colcon
   extensions recognize `ament_cargo` packages and resolve `rclrs`/`dc_interfaces`/
   `std_msgs`/`std_srvs` to the workspace's generated crates or crates.io.
2. `vcs import src < ros2_data_collection_jazzy.repos` into the workspace `src/`
   alongside this repo, *before* `rosdep install`/`colcon build` (matching the existing
   `docker/ci/Dockerfile` and `docker/source/Dockerfile` convention for extra source
   deps). This pulls in the message repos and the `rosidl_rust` code generator, since
   Jazzy doesn't yet ship `rosidl_generator_rs` and pre-built apt message packages carry
   no Rust bindings — only source-built ones do. `rosidl_rust` is pinned to the exact
   commit this package was verified against, since it and `rosidl_runtime_rs` (pinned
   separately in `dc_bridge/Cargo.lock` — see the version-skew bug below) are developed
   in lockstep and have drifted out of sync before.
3. `colcon build --packages-up-to dc_bridge` then builds like any other package in this
   workspace, generating `lib/dc_bridge/dc_bridge`.

Note that `rosdep install --from-paths src --ignore-src` will still report
`rosidl_runtime_rs` as unresolvable — that's expected, not a setup mistake. Neither it
nor `rclrs` have (or ever will have) a rosdep key: they're plain Cargo/crates.io
dependencies that colcon-ros-cargo resolves through its own Cargo-registry patching,
entirely bypassing rosdep. `<depend>` tags for them in `package.xml` exist only so
colcon knows about the dependency for build ordering. rosdep's error summary also only
ever surfaces one unresolved key per package even when several are unresolvable, so
don't read "only one reported" as "the others resolved."

Step 1 (the Rust/cargo toolchain itself, plus the colcon-cargo/colcon-ros-cargo
extensions) still isn't wired into this repo's own Docker images or CI (this is the
first Rust package; see ADR-0004) — only step 2's source-import path is. Adding Rust to
`docker/source/Dockerfile` (conditionally, since `humble` images have no Rust package to
build) is a follow-up, not part of this tracer bullet.

## Runtime flow (ADR-0006)

The Bridge is a plain node outside the lifecycle manager. Startup order is: Vector
(Supervisor spawns it with a generated Fluent-Forward-source / console-sink config) →
Bridge subscribes and starts forwarding → the readiness service starts answering `true`
once a TCP probe against Vector's Forward port succeeds → the lifecycle manager can
gate activating collection nodes on that. If Vector dies, the Supervisor respawns it and
readiness drops to `false` until the new process is accepting connections again.
