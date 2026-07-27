# The Bridge returns to C++; the Rust pilot (ADR-0004) is reverted

**Supersedes:** ADR-0004 (Rust Bridge pilot).

ADR-0004 made the new `dc_bridge` node Rust (`rclrs`) as a deliberately contained pilot
— its own words: *"We want first-party Rust in the project."* The bet was explicitly
reversible: *"if not, the loss is one small package."* We are exercising that exit
clause. The Bridge is now plain C++ (`ament_cmake`, `rclcpp`); everything else about the
DC 2.0 architecture (external Vector shipper, shipper ingest protocol boundary, blessed
Destinations + passthrough, the Uploader's verify-then-delete semantics) is unchanged.

## Why

The pilot's stated goal was to prove that first-party Rust could live in DC at low risk.
On ROS 2 Jazzy, today, it does not clear that bar — the cost is in the toolchain, not the
Bridge's own logic:

- **`ros2_rust` has no stable Jazzy release.** Building `dc_bridge` required
  `vcs`-importing the ROS message repos *and* the `rosidl_rust` code generator pinned to
  a specific git commit (Jazzy ships no `rosidl_generator_rs`, and pre-built apt message
  packages carry no Rust bindings). The generator and `rosidl_runtime_rs` are developed
  in lockstep and had already drifted out of sync once, forcing a `[patch.crates-io]`.
- **`colcon-ros-cargo` link-flag generation is incomplete.** A full-workspace build
  (as opposed to the narrower `--packages-up-to dc_bridge` every pilot PR verified with)
  made sibling interface packages "discoverable" via the ament index, at which point
  `rclrs`'s build script emitted `-l<pkg>__rosidl_typesupport_c` flags for packages
  nothing supplied a matching `-L` for — surfacing as opaque "unable to find library"
  link failures that took real debugging to root-cause and only "fixed" by
  `COLCON_IGNORE`-ing packages back out of the index.
- **Every CI/dev environment paid for it.** The image needed rustup, `colcon-cargo`/
  `colcon-ros-cargo`, `libclang`, the source-built message repos, and a stack of
  `COLCON_IGNORE`/`--skip-keys` workarounds — none of which are about what the Bridge
  *does*.

Against that, none of the Bridge's actual responsibilities need Rust. Talking to Vector
is the shipper ingest protocol (msgpack over a TCP socket); process supervision is
`fork`/`exec`/`waitpid` + `PR_SET_PDEATHSIG`; config rendering is string/TOML
generation; the File uploads (ADR-0005) are S3 multipart, which the AWS SDK for C++
provides directly. The Humble line did S3 uploads with a single `minio-go` call and no
resumability at all, so nothing here is beyond C++.

The decisive factor is **developer experience, of which the CI simplification is only
the visible symptom**: as C++, `dc_bridge` is an ordinary `ament_cmake` package. A
contributor clones the repo and runs `rosdep install && colcon build` — the same as
every other `dc_*` package. That plug-and-play property is exactly what the pilot was
meant to test for Rust and, on Jazzy today, could not deliver.

## Decision

- `dc_bridge` is `ament_cmake` C++ (`rclcpp`). The ROS-independent core (Forwarder,
  Supervisor, Readiness, TopicConfig, ConfigRenderer) is a plain library, unit-tested
  with `gtest` and buildable/testable without a ROS install — preserving the
  `dc_bridge_core` "pure logic, no ROS needed to test it" property the Rust design had.
- Dependencies are all rosdep-resolvable: `rclcpp`, `dc_interfaces`, `std_msgs`,
  `std_srvs`, `nlohmann-json-dev`, plus `tomlplusplus`/`msgpack-cxx` (header-only C++
  libraries) via a small repo-local rosdep source (`rosdep/dc.yaml`) since upstream
  rosdistro has no key for them.
- The File Uploader (ADR-0005) uses the AWS SDK for C++ against S3-compatible object
  storage, verified working against RustFS (PutObject + multipart) before adoption.

## Consequences

- Rust, rustup, `colcon-cargo`/`colcon-ros-cargo`, `libclang`, the source-built message
  repos (`ros2_data_collection_jazzy.repos`), and every `COLCON_IGNORE`/`--skip-keys`
  Rust workaround leave the tree. The toolchain is smaller than the Rust pilot's *and*
  than Humble's (Go and the Fluent Bit C fork were already gone per ADR-0001).
- Signal handling is simpler: `rclcpp` handles SIGINT/SIGTERM and returns from `spin()`,
  and `on_shutdown` stops the supervised Vector — the Rust node needed an explicit
  `ctrlc` handler to avoid orphaning Vector because `rclrs::Context::ok()` never returned
  false.
- First-party Rust in DC is deferred, not foreclosed. If `ros2_rust` matures to a stable
  distro release with reliable `colcon` integration, the experiment can be revisited from
  a working C++ baseline — the same component-by-component path ADR-0004 imagined, just
  in the other direction for now.
