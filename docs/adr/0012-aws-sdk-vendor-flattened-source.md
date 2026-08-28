# aws_sdk_vendor builds from a flattened, plain-committed source tree

`aws_sdk_vendor`'s `CMakeLists.txt` built aws-sdk-cpp (ADR-0007) via
`ExternalProject_Add`'s `GIT_REPOSITORY`/`GIT_TAG` — a live `git clone` of
`aws-sdk-cpp` at build time, plus a hand-rolled `PATCH_COMMAND` recursing into its
`crt/aws-crt-cpp` submodule. The ROS buildfarm's binarydeb jobs run with no network
access, so this fails there unconditionally; #423's `vendor-network-check` build
stage proved it with a clear, attributable network error against `main`. #421/#134
need this fixed before DC can be released via apt at all.

**Decision: vendor a flattened, plain-committed source tree, not a git submodule.**

`aws_sdk_vendor/vendor/aws-sdk-cpp/` now holds the actual source AWS ships at tag
`1.11.600` — `git clone`d, submodules resolved by hand, then every `.git`/
`.gitmodules` stripped and the result committed as plain files. Not a git submodule:
bloom's release-tarball export (`vcstools.git_archive_all.GitArchiver`, reading the
working tree, not `git archive`'s blob-store plumbing) does not reliably preserve
submodule content, the same reason `vector_vendor`'s amendment (ADR-0002) avoided
submodules for its own vendored artifact. `CMakeLists.txt`'s `ExternalProject_Add`
now takes a plain `SOURCE_DIR` pointing into the vendored tree with no `GIT_*`/`URL`/
`DOWNLOAD_COMMAND` option at all — CMake infers "no download step" from that
combination, so the network is never touched.

## Pruned to what `-DBUILD_ONLY=s3` actually builds

A straight `git clone` of aws-sdk-cpp at this tag is **1.2GB** (415 generated
per-service client directories, the Java/Maven codegen tooling under `tools/`, the
full test suites) plus **377MB** for the recursively-cloned `crt/aws-crt-cpp`
dependency chain (13 submodules, dominated by `aws-lc` at 287MB) — committing that
whole tree would roughly double this repo's size for code the build never touches.
Tracing `cmake/sdks.cmake`/`cmake/sdksCommon.cmake` shows `BUILD_ONLY=s3` with
`ENABLE_TESTING=OFF` resolves to exactly `{s3, core}` (`s3` has no entry in
`SDK_DEPENDENCY_LIST`, so it gets only the implicit `core` dependency, and the
test-dependency map is never consulted with testing off) — none of the other 414
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
compiles (plus each dependency's own `LICENSE`/`NOTICE`, kept for attribution —
`REUSE.toml` covers the tree with one blanket `Apache-2.0` annotation rather than
auditing tens of thousands of individual upstream files, most of which already carry
their own SPDX header anyway).

The one submodule content exclusion from the old `PATCH_COMMAND` carries over
unchanged: `crt/s2n/tests/cbmc/aws-verification-model-for-libcrypto`, s2n's CBMC
formal-verification model, needed only for `make cbmc` and irrelevant to
`-DBUILD_ONLY=s3` — simply never cloned when the vendored tree was assembled, since
there is no submodule-update step left to skip it from.

## Measured build time (#423's `--network none` check)

`tools/e2e/scripts/verify_network_isolation.sh` (the `vendor-network-check` stage)
now **passes**: `colcon build --packages-select aws_sdk_vendor` finishes in
**4 minutes 29 seconds** under `--network=none`, entirely local compilation with no
download step. That compares to the ~20-minute network-*and*-build baseline recorded
against the old `ExternalProject_Add` in `tools/e2e/Containerfile`/#271 — expected,
since that baseline paid for a full-history-adjacent shallow clone, submodule
resolution, and the same compile in one figure, where this build now pays for the
compile alone. No hard pass/fail threshold is set here (the buildfarm's own job
timeout isn't publicly discoverable), this number is informational.

## Consequences

- `aws_sdk_vendor/CMakeLists.txt` performs no network I/O; `package.xml` drops its
  `git` `build_depend` (nothing left that invokes it).
- Bumping `AWS_SDK_VERSION` is no longer a one-line change: it means re-running the
  clone/submodule/prune steps above against the new tag and re-committing the result
  — the version pin comment in `CMakeLists.txt` says so directly.
- `.pre-commit-config.yaml`'s global `exclude` now also skips
  `aws_sdk_vendor/vendor/.*`: upstream AWS source shouldn't be reformatted,
  spell-checked, or otherwise rewritten by this repo's hooks — the same rationale
  already applied to `LICENSES/`.
- `tools/e2e/Containerfile`'s `vendor-network-check` stage, previously documented as
  "expected to fail" pending this issue, now passes — its comments (and this script's)
  are updated to say so instead of describing a still-future fix.

## Considered Options

- Move `aws_sdk_vendor`'s source into its own repo, pulled via `vcs import` like
  `vector_vendor` (ADR-0002's amendment) — rejected: that amendment's motivating
  constraint was a single large binary blob that git never deduplicates across
  version bumps (~106MB/bump against a 1GB/month free LFS budget). Pruned source text
  compresses and deltas normally in git; #425 asks for it flattened and committed
  here, not relocated.
- Keep the git submodule aws-sdk-cpp itself uses for `crt/aws-crt-cpp` — rejected:
  bloom's release-tarball export does not reliably preserve submodule content (the
  same finding that ruled out a submodule for `vector_vendor`), and the ROS buildfarm
  needs the source present verbatim in the release tarball it builds from.
- Vendor the full, unpruned clone (1.6GB) for simplicity — rejected: it would roughly
  double this repository's size for 415 service clients and a dependency (`aws-lc`)
  the build's own `CMakeLists.txt` logic never reaches, for no benefit over the
  pruned tree the same build produces identical output from.
