# aws_sdk_vendor fetches aws-sdk-cpp live from GitHub; the flattened-source design is rejected

`aws_sdk_vendor` builds aws-sdk-cpp (core + s3 only) by cloning
`github.com/aws/aws-sdk-cpp.git` directly at `colcon build` time, via
`ament_cmake_vendor_package`'s `ament_vendor()` macro (`VCS_TYPE git`) — the same idiom
`zmqpp_vendor`/`tinyxml_vendor`/`yaml_cpp_vendor` already use successfully on the real
ROS buildfarm. No aws-sdk-cpp source is vendored anywhere: `aws_sdk_vendor` itself lives
in its own repo, `github.com/Minipada/aws_sdk_vendor` (mirroring `vector_vendor`'s split,
ADR-0002's amendment), but that repo now holds only this package's `CMakeLists.txt` and
`package.xml` — a thin recipe, not a vendored source tree.

## Why

An earlier design (#425, worked out in PR #433) kept `aws_sdk_vendor` in its own repo but
flattened aws-sdk-cpp's pinned source into it, plain-committed and pruned to exactly what
`-DBUILD_ONLY=s3` compiles, so no network fetch happened at `colcon build` time at all.
That design rested on one assumption: that ROS buildfarm binarydeb jobs run with no
network access, the same constraint that motivated `vector_vendor`'s checked-in-binary
amendment (see ADR-0002).

That assumption is false for the actual compilation step. `ros_buildfarm`'s own
job-generation source
(`ros_buildfarm/templates/release/deb/binarypkg_job.xml.em`, the "Run Dockerfile - build
binarydeb" section) invokes `docker run --net=host` for the container that runs the
build. And this isn't theoretical: `zmqpp_vendor` — which does exactly this
live-git-clone-at-build-time pattern via `ament_vendor()` — has a real, currently
succeeding Jenkins job on build.ros2.org
(`Jbin_uN64__zmqpp_vendor__ubuntu_noble_amd64__binary`, build #12, `SUCCESS`).
`vector_vendor`'s problem was never generic buildfarm network isolation; it was Vector
shipping as a large pinned binary rather than buildable source, which the buildfarm has
no story for regardless of network access, and which `ament_vendor()` doesn't help with
either.

Given real network access, a live fetch is simpler than a flattened source and needs no
vendored tree, no separate REUSE audit of ~2600 upstream files, and no manual re-pruning
on every AWS SDK version bump. `crt/aws-crt-cpp`'s own submodule chain (13 submodules,
one of which — `s2n`'s CBMC formal-verification model — is unrelated to
`-DBUILD_ONLY=s3`) can be let recurse in full via `ament_vendor()`'s `vcs import
--recursive`: with real network available, the one extra unused clone is harmless, and
`vcs import` has no mechanism to exclude a single nested submodule anyway.

`aws_sdk_vendor` staying in its own repo, rather than moving back into
`ros2_data_collection`, is a separate call from the fetch-mechanism question above: this
package's own content (`CMakeLists.txt` + `package.xml`) is small either way, so unlike
`vector_vendor`'s split — which exists specifically to keep ~106MB/bump of binary growth
out of `ros2_data_collection`'s git history — there's no bloat this split avoids. It
stays split anyway so every vendor package this workspace pulls in via `.repos` follows
the same layout and the same bump/release workflow.

## Decision

- `github.com/Minipada/aws_sdk_vendor`'s `CMakeLists.txt` calls `ament_vendor()` with
  `VCS_TYPE git`, `VCS_URL https://github.com/aws/aws-sdk-cpp.git`, and `VCS_VERSION`
  pinned to the same tag ADR-0007's Phase 2 verified against RustFS. `GLOBAL_HOOK` is
  required: `dc_bridge` (the real downstream consumer) only ever calls
  `find_package(AWSSDK)`, never `find_package(aws_sdk_vendor)` first, so the install
  prefix has to reach `CMAKE_PREFIX_PATH` via an environment hook applied
  unconditionally, not a CMake config-extra a consumer would only pick up by
  `find_package()`-ing this vendor package itself.
- The flattened `vendor/aws-sdk-cpp/` tree #425/PR #433 committed into
  `github.com/Minipada/aws_sdk_vendor` is removed from that repo; it now holds only the
  thin `ament_vendor()` recipe. `ros2_data_collection.repos` gains an `aws_sdk_vendor`
  entry (pinned to a tag, alongside `vector_vendor`'s), the same `vcs import` path
  `vector_vendor` already uses.
- `tools/e2e/Containerfile`'s `vendor-network-check` stage keeps proving the *lack* of
  network isolation is expected for `aws_sdk_vendor`: under `--network=none` its `colcon
  build` is supposed to fail, same as it always has, since nothing about this decision
  changes what the buildfarm actually does for this package.

## Consequences

- No aws-sdk-cpp source lives in `ros2_data_collection`, in `Minipada/aws_sdk_vendor`,
  or anywhere else — one less REUSE audit surface, and no manual pruning step on every
  AWS SDK version bump (just move `AWS_SDK_VERSION`).
- `aws_sdk_vendor`'s `colcon build` step needs network, same as it always has since
  ADR-0007; `toolchain-base`'s `vcs import` (which fetches both `vector_vendor` and
  `aws_sdk_vendor`) stays isolated from `tools/e2e/Containerfile`'s own build steps, and
  the network-isolated check (#423) keeps demonstrating — rather than trying to
  eliminate — that isolation boundary.
- Bumping the AWS SDK version now touches two repos, same as bumping Vector already
  does: land the new `AWS_SDK_VERSION`/`<version>`/tag in `Minipada/aws_sdk_vendor`,
  then bump the pinned `version:` in `ros2_data_collection.repos`.
- If a future ROS buildfarm policy change *does* restrict binarydeb network access (the
  false premise here becoming true later), this decision reverses again; nothing about
  the flattened-source design explored in PR #433 is lost — it is documented there and
  in this ADR's history, not deleted.
