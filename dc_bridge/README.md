# dc_bridge

The Bridge (ADRs 0001/0004/0006): a Rust (`rclrs`) node that subscribes to `dc_interfaces/msg/StringStamped`
Record topics and forwards every Record to the Vector shipper over the Fluent Forward
protocol. It renders Vector's configuration from ROS parameters for the blessed
Destination set (`postgres`, `s3`, `file`, `console` — ADR-0003) and passes raw Vector
snippets (`custom_config_files`) through for everything else in Vector's sink catalog,
via the public per-Tag `dc.<tag>` routes documented in `doc/src/dc/destinations.md`.

## Layout

- **`dc_bridge_core/`** — the ROS-independent core: `Forwarder` (msgpack framing, socket
  lifecycle, reconnection, backpressure), `Supervisor` (generic process respawn),
  `Readiness` (TCP-accept probe), `config` (topic → Fluent Forward tag), `vector_config`
  (locates the vendored binary), and `render` (ADR-0003's config renderer — see below).
  It's a standalone crate with its own `Cargo.toml` (declaring `[workspace]` so it never
  gets swept into the outer package's workspace) and zero ROS dependencies, so it builds
  and tests with plain Cargo:

  ```sh
  cd dc_bridge/dc_bridge_core
  cargo test
  ```

  This is how the Forwarder/Supervisor/Readiness acceptance criteria for this package
  were verified in a sandbox with no ROS 2 or ros2_rust install available — no `colcon`
  or `rclrs` toolchain is required to run these tests.

- **`src/main.rs`** — the actual ROS node. It declares `shipper.*`/`destinations`/
  `<name>.*` parameters (ADR-0003's config contract — see below), `vector_forward_host`,
  `vector_forward_port`, and `vector_binary`; renders the full Vector config via
  `dc_bridge_core::render` and hands it to the Supervisor (which supervises the vendored
  `vector_vendor` binary); subscribes to the union of every configured Destination's
  `inputs` topics; exposes a `~/ready` (`std_srvs/Trigger`) readiness service; and
  installs a SIGINT/SIGTERM handler that stops Vector before exiting (see "Runtime flow"
  below — rclrs doesn't do this on its own). It depends on `rclrs`, `dc_interfaces`,
  `std_msgs`, and `std_srvs`, all only resolvable inside a colcon workspace (see below).

- **`tests/end_to_end.rs`** — `colcon test`-integrated integration tests (colcon-ros-cargo
  runs `cargo test` + `cargo fmt --check` for `ament_cargo` packages, same as any other
  Cargo package) that spawn the *real* `dc_bridge` binary and `rclrs`-drive a test
  client against it: readiness with a postgres destination; the supervised Vector
  process stopping on SIGTERM; a Record landing as a real Postgres row; a Record
  landing as an object in an S3-compatible store's bucket configured purely via ROS
  params (verified against RustFS, the recommended self-hosted store now that MinIO's
  community edition is archived upstream); a `custom_config_files` snippet shipping
  Records to an un-blessed `http` sink by consuming the public `dc.<tag>` route
  (self-contained — the test plays the HTTP server); and a colliding snippet failing
  dc_bridge startup loudly. The store-backed tests **require** a Postgres at
  127.0.0.1:5432 and an S3-compatible store at 127.0.0.1:9000 and hard-fail
  immediately with setup instructions when they're missing — they deliberately never
  skip, since libtest swallows passing tests' output and a skipped run is
  indistinguishable from a verified one. This file is what caught the signal-handling
  bug below — manual verification alone had missed it.

## Config renderer (ADR-0003)

`dc_bridge_core::render` is a pure, I/O-free module (`src/render.rs`) mapping ROS
parameters to a complete Vector configuration. Given the unified-destinations config
contract:

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
      password: "$DC_PG_PASSWORD"   # env expansion, consistent with existing $HOME usage
      database: "dc"
      table: "dc"
      time_key: "date"
      time_format: "double"      # double | iso8601
```

it renders: the Fluent Forward source; one `remap` (VRL) transform that normalizes each
blessed destination's timestamp field into `time_key` per `time_format` — `double`
(Unix epoch seconds as a float) or `iso8601` (`%Y-%m-%dT%H:%M:%S%.9f`), replacing the
Humble-era chained Lua filters (`dc_destinations/include/dc_destinations/
flb_destination.hpp` on the `humble` branch); one `route` transform named `dc` with one
branch per distinct Tag across every Destination's `inputs`, so each Tag's Records get a
stable public output at `dc.<tag>` (ADR-0003's passthrough contract, documented as API
in `doc/src/dc/destinations.md`) that blessed sinks and `custom_config_files` snippets
alike consume; a disk buffer (`shipper.data_dir`, minimum ~256 MiB per Vector's own
`postgres` sink constraint) per persistent sink; and the sinks themselves. All four
blessed types are implemented: `postgres`, `s3` (AWS, or a RustFS/MinIO-style custom `endpoint`),
`file`, and `console` (no disk buffer — it's a debugging sink).

Two layers, both pure and gold-file-tested (`src/render.rs`'s `#[cfg(test)]` module,
comparing against checked-in fixtures under `tests/fixtures/render/*.toml` via parsed
`toml::Table` equality so incidental formatting differences don't make the tests
brittle — every fixture is also `vector validate`d against the real pinned binary
whenever it changes):

- `destination_from_raw` (with the per-type `*Params::from_raw` constructors) validates
  the flat, stringly-typed values ROS parameters naturally give (`<name>.type`,
  `<name>.host`, …) into typed config — every "invalid-parameter rejection" acceptance
  criterion (unknown `type`, bad `time_format`/`receives`, missing required field, half
  an S3 credential pair, out-of-range `port`, empty `inputs`/`destinations`,
  duplicate/invalid/reserved destination names, an undersized disk buffer) is a
  `RenderError` variant with its own test.
- `render` turns already-validated `RenderConfig` into the Vector TOML text above.

`validate_custom_config_files` (also pure) checks the passthrough side: every
`custom_config_files` snippet must parse as TOML, define at least one component, and
not claim a component id owned by the rendered config or another snippet — so a bad
snippet fails loudly at Bridge startup with an error naming the file. As a backstop,
`main.rs` also runs `vector validate --no-environment` over the merged `--config` set
(catching e.g. a snippet consuming a `dc.<tag>` route no Destination subscribes) before
the Supervisor ever starts Vector.

`expand_env` (also pure — it takes an injected lookup function rather than reading the
real environment) implements the `$NAME`/`${NAME}` expansion `shipper.data_dir`,
destination credentials (`password`, `secret_access_key`), and `custom_config_files`
paths use; `main.rs` is the only place that calls it with `std::env::var`.

Run just these tests (no ROS, Vector, or Postgres needed):

```sh
cd dc_bridge/dc_bridge_core
cargo test render::
```

## Verified in a real ROS 2 Jazzy + ros2_rust environment

`colcon build --packages-select dc_bridge` succeeds, and `colcon test --packages-select
dc_bridge` passes (2/2, run repeatedly with no flakiness once the test correctly
disambiguated its own Vector process — see below), in a Docker container
(`ros:jazzy-ros-base` + rustup + `colcon-cargo`/`colcon-ros-cargo` + the Jazzy message
repos + `rosidl_rust`, per the recipe below). `ros2 run dc_bridge dc_bridge` locates and
spawns the vendored Vector binary, the `~/ready` service answers `true` once Vector is
listening, and a `StringStamped` published on a subscribed topic shows up on Vector's
console sink with the derived tag, e.g. publishing `{data: '{"uptime_s": 42}'}` on
`/dc/measurement/uptime` produced:

```
{"host":"127.0.0.1","source_type":"fluent","tag":"dc.measurement.uptime","timestamp":"1970-01-01T00:00:00Z","uptime_s":42}
```

Killing the real (vendored) Vector process was also confirmed to trigger an automatic
respawn (a new PID, Vector re-listening) via the Supervisor, and readiness recovers
to `true` afterward.

Three real bugs surfaced only in this environment (unit tests can't catch any of them,
since they're specific to the generated/real rclrs API surface, or to dc_bridge's own
process-supervision behavior under signals):

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
- **Vector orphaned on shutdown**: `rclrs::Context::ok()` unconditionally returns `true`
  today (no signal handling yet), so `executor.spin()` never returns on Ctrl-C,
  `ros2 launch` shutdown, or a plain `kill <pid>` — the process just dies via the OS
  default, and the Supervisor's `Drop` (which would stop Vector) never runs since it
  lives in a detached background thread that loops forever. Fixed by installing an
  explicit `ctrlc` handler (the `termination` feature, not just default SIGINT) that
  stops Vector before exiting. SIGKILL still can't be caught by any userspace handler —
  that part is an inherent, unavoidable Unix limit, not specific to this.
- **Vector orphaned by a SIGTERM during startup**: the handler above used to be
  installed only *after* `Supervisor::start()`, so a signal landing in between (hit
  intermittently by `stops_the_supervised_vector_process_on_sigterm` once the startup
  sequence also grew a `vector validate` step) killed dc_bridge via the OS default and
  left the just-spawned Vector running. Fixed by installing the handler before the
  first `start()` and making `Supervisor::start` a no-op after `stop()`, so the handler
  can fire at any point of the startup sequence without leaking a process.

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

On shutdown, a SIGINT/SIGTERM handler explicitly stops Vector before the process exits
(see the "Vector orphaned on shutdown" bug above) — dc_bridge doesn't rely on Rust's
`Drop` running here, since the Supervisor lives in a detached background thread that
would otherwise never unwind.
