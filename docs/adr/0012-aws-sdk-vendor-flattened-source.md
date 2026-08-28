# aws_sdk_vendor splits into its own repo, building from flattened source

`aws_sdk_vendor`'s `CMakeLists.txt` built aws-sdk-cpp (ADR-0007) via
`ExternalProject_Add`'s `GIT_REPOSITORY`/`GIT_TAG` — a live `git clone` of
`aws-sdk-cpp` at build time, plus a hand-rolled `PATCH_COMMAND` recursing into its
`crt/aws-crt-cpp` submodule. The ROS buildfarm's binarydeb jobs run with no network
access, so this fails there unconditionally; #423's `vendor-network-check` build
stage proved it with a clear, attributable network error against `main`. #421/#134
need this fixed before DC can be released via apt at all.

**Decision: `aws_sdk_vendor` moves to [github.com/Minipada/aws_sdk_vendor](https://github.com/Minipada/aws_sdk_vendor)
(default branch `jazzy`), the same move `vector_vendor` already made (ADR-0002's
amendment) and for the same reason** — pulled into this workspace via
`ros2_data_collection.repos` (vcstool), pinned to a tag (`v1.11.600`), not a branch,
so a given `ros2_data_collection` commit always resolves the same content.
`tools/e2e/Containerfile`'s `toolchain-base` stage runs the `vcs import` (network
required, same footing as the rosdep install right below it); every downstream
`colcon build` — including #423's `--network=none` check — builds against the
already-fetched result, unaffected by where the source physically came from.

The flattened-source decision itself (below) doesn't change by moving repos — only
*where* that ~100MB tree's growth lands. Keeping it inside `ros2_data_collection`
was #425's original plan, until it became clear the same 106MB-per-Vector-bump
argument that moved `vector_vendor` out applies just as much here: an AWS SDK version
bump replaces most of the vendored tree, and that growth doesn't need to be
`ros2_data_collection`'s own git history's cost to carry, when contributors working
on the pipeline itself have no reason to pay it on every clone.

## Flattened, plain-committed source — not a git submodule

`aws_sdk_vendor`'s own repo holds the actual source AWS ships at tag `1.11.600` under
`vendor/aws-sdk-cpp/` — `git clone`d, submodules resolved by hand, then every `.git`/
`.gitmodules` stripped and the result committed as plain files. Not a git submodule:
bloom's release-tarball export (`vcstools.git_archive_all.GitArchiver`, reading the
working tree, not `git archive`'s blob-store plumbing) does not reliably preserve
submodule content, the same reason `vector_vendor` avoided one for its own vendored
artifact.

## Built via `ament_cmake_vendor_package`, not a hand-rolled `ExternalProject_Add`

`CMakeLists.txt` uses `ament_cmake_vendor_package`'s `ament_vendor()` macro — the same
one dozens of other ROS `_vendor` packages use (`zmqpp_vendor`, `tinyxml_vendor`,
`yaml_cpp_vendor`, …) — instead of a bespoke `ExternalProject_Add` call:

```cmake
ament_vendor(aws_sdk_vendor
  VCS_TYPE path
  VCS_URL vendor/aws-sdk-cpp
  GLOBAL_HOOK
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_ONLY=s3
    -DENABLE_TESTING=OFF
    -DAUTORUN_UNIT_TESTS=OFF
    -DENABLE_ZLIB_REQUEST_COMPRESSION=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
)
```

`VCS_TYPE path` is the macro's own answer to "vendor pre-fetched source with zero
network at build time": it hands `ExternalProject_Add` a plain `SOURCE_DIR` with no
`GIT_*`/`URL`/`DOWNLOAD_COMMAND` option, so the download step is disabled entirely.
Two non-obvious flags were needed to make a real downstream consumer (`dc_bridge`)
actually build against the result, both found by testing the integration end-to-end
rather than trusting the vendor package's own isolated build to prove it:

- **`GLOBAL_HOOK`** — without it, the macro only widens `CMAKE_PREFIX_PATH` via a
  `CONFIG_EXTRAS` script that runs when a consumer calls `find_package(aws_sdk_vendor)`
  first. `dc_bridge` calls plain `find_package(AWSSDK COMPONENTS s3)` with no such
  call, and failed with "Could not find AWSSDKConfig.cmake" until `GLOBAL_HOOK` was
  added — it instead registers a plain ament environment hook, sourced for every
  downstream package in the workspace regardless of what it `find_package()`s.
- **`-DCMAKE_BUILD_TYPE=Release`** — colcon builds `aws_sdk_vendor`'s own top-level
  `CMakeLists.txt` with `CMAKE_BUILD_TYPE` defined but empty; `ament_vendor()`'s
  cache-seeding logic propagates that as an `INTERNAL` cache entry into the
  `ExternalProject`, which blocks aws-sdk-cpp's own non-`FORCE`
  `set(CMAKE_BUILD_TYPE "Release" CACHE STRING ...)` default from ever taking effect.
  Left empty, every `.so` still builds, but CMake's per-config export generation
  silently drops `IMPORTED_LOCATION` from the installed `*-targets.cmake` files —
  `find_package(AWSSDK)` itself still succeeds, but `dc_bridge`'s CMake generate step
  then fails on every CRT target ("IMPORTED_LOCATION or IMPORTED_IMPLIB not set").
  Root-caused by inspecting the installed `aws-c-common-targets.cmake` directly: its
  `file(GLOB ... targets-*.cmake)` found zero per-config files with the flag missing.

Verified end to end, not just at the vendor package's own build: a full
`colcon build --packages-up-to dc_bridge` against a from-scratch workspace succeeds,
and `ldd` against the built `dc_bridge` binary (after sourcing `install/setup.sh`)
resolves every AWS SDK/CRT `.so` correctly at runtime.

## Pruned to what `-DBUILD_ONLY=s3` actually builds

A straight `git clone` of aws-sdk-cpp at this tag is **1.2GB** (415 generated
per-service client directories, the Java/Maven codegen tooling under `tools/`, the
full test suites) plus **377MB** for the recursively-cloned `crt/aws-crt-cpp`
dependency chain (13 submodules, dominated by `aws-lc` at 287MB) — committing that
whole tree would roughly double the vendor package's own repo size for code the build
never touches. Tracing `cmake/sdks.cmake`/`cmake/sdksCommon.cmake` shows
`BUILD_ONLY=s3` with `ENABLE_TESTING=OFF` resolves to exactly `{s3, core}` (`s3` has
no entry in `SDK_DEPENDENCY_LIST`, so it gets only the implicit `core` dependency, and
the test-dependency map is never consulted with testing off) — none of the other 414
generated service clients are reachable from the build at all. Separately,
`crt/aws-crt-cpp/CMakeLists.txt` only adds `crt/aws-lc` when `NOT USE_OPENSSL`;
`aws_sdk_vendor` never overrides `USE_OPENSSL` from its default `ON`, so
`aws-c-cal` links the system's `libcrypto` (the package.xml `libssl-dev` dependency)
and AWS-LC is never built — confirmed by tracing `aws-c-cal/CMakeLists.txt`'s
`USE_OPENSSL` branch, not by running the build with it present and checking the log.

The vendored tree is pruned to match: `generated/src/` keeps only
`aws-cpp-sdk-s3`, `generated/tests`/`generated/protocol-tests`/`generated/smoke-tests`/
top-level `tests/`/`tools/` (codegen-only) are gone, and `crt/aws-lc` is deleted
entirely. `crt/s2n` stays — it's built unconditionally on
`UNIX AND NOT APPLE AND NOT BYO_CRYPTO` regardless of `USE_OPENSSL`, since s2n
implements the TLS handshake itself and only borrows libcrypto for primitives. Net
result: **1.2GB + 377MB → ~100MB** committed, all of it source the build actually
compiles (plus each dependency's own `LICENSE`/`NOTICE`, kept for attribution — the
vendor repo's own `REUSE.toml` covers the tree with one blanket `Apache-2.0`
annotation rather than auditing tens of thousands of individual upstream files, most
of which already carry their own SPDX header anyway).

The one submodule content exclusion from the old `PATCH_COMMAND` carries over
unchanged: `crt/s2n/tests/cbmc/aws-verification-model-for-libcrypto`, s2n's CBMC
formal-verification model, needed only for `make cbmc` and irrelevant to
`-DBUILD_ONLY=s3` — simply never cloned when the vendored tree was assembled, since
there is no submodule-update step left to skip it from.

## Measured build time (#423's `--network none` check)

`tools/e2e/scripts/verify_network_isolation.sh` (the `vendor-network-check` stage)
now **passes**: `colcon build --packages-select aws_sdk_vendor` finishes in
**3-5 minutes** under `--network=none` (measured 4m29s and 3m5s across repeated runs;
variance comes from container-layer cache state, not the build itself), entirely
local compilation with no download step. That compares to the ~20-minute
network-*and*-build baseline recorded against the old `ExternalProject_Add` in
`tools/e2e/Containerfile`/#271 — expected, since that baseline paid for a
full-history-adjacent shallow clone, submodule resolution, and the same compile in
one figure, where this build now pays for the compile alone. No hard pass/fail
threshold is set here (the buildfarm's own job timeout isn't publicly discoverable),
this number is informational.

## Consequences

- `ros2_data_collection` no longer has an `aws_sdk_vendor/` directory at all —
  `ros2_data_collection.repos` gains an entry pinning it to `v1.11.600`, resolved the
  same way `vector_vendor` already is.
- `tools/e2e/Containerfile`'s `toolchain`/`vendor-network-check` stages no longer
  `COPY aws_sdk_vendor src/ros2_data_collection/aws_sdk_vendor` — that source now
  lands via `toolchain-base`'s existing `vcs import` step, alongside `vector_vendor`.
- `.pre-commit-config.yaml`'s global `exclude` no longer needs an
  `aws_sdk_vendor/vendor/.*` entry — there's nothing under that path in this repo to
  exclude.
- Bumping `AWS_SDK_VERSION` now touches two repos: land the new pruned tree + tag in
  `aws_sdk_vendor` (steps documented in that repo's own README), then bump the pinned
  `version:` in `ros2_data_collection.repos` to match — the same two-repo dance
  `vector_vendor` bumps already follow.
- This is a development-workspace convenience only (`vcs import`/`colcon build` from
  source, per ADR-0002's own amendment); it doesn't make `aws_sdk_vendor` installable
  via `apt` on its own — that needs an independent `bloom-release`, not yet done.

## Considered Options

- Keep the flattened source committed inside `ros2_data_collection` itself (#425's
  original scope) — reconsidered mid-implementation: nothing about the rest of this
  repo's history needs to grow every time the AWS SDK ships a version bump, the exact
  reasoning that already moved `vector_vendor` out.
- Keep the git submodule aws-sdk-cpp itself uses for `crt/aws-crt-cpp` — rejected:
  bloom's release-tarball export does not reliably preserve submodule content (the
  same finding that ruled out a submodule for `vector_vendor`), and the ROS buildfarm
  needs the source present verbatim in the release tarball it builds from.
- Vendor the full, unpruned clone (1.6GB) for simplicity — rejected: it would roughly
  double the vendor repo's size for 415 service clients and a dependency (`aws-lc`)
  the build's own `CMakeLists.txt` logic never reaches, for no benefit over the
  pruned tree the same build produces identical output from.
- Hand-roll `ExternalProject_Add` directly (the original draft of this ADR) instead of
  `ament_cmake_vendor_package`'s `ament_vendor()` macro — rejected once pointed at the
  established idiom: it's the same macro already proven across many ROS `_vendor`
  packages, needs less custom CMake to maintain, and (via `VCS_TYPE path`) supports
  exactly the "pre-fetched local source, no network" case this issue needs.
