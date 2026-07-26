# Zero-loss E2E harness (#249)

Proves the DC 2.0 (Jazzy) pipeline's reliability claims end-to-end: launch-to-first-
Record under 10 seconds, and zero Record/File loss across a destination outage plus a
full stack restart. Runs with **Podman** (see `CLAUDE.md` "Containers: Podman, not
Docker") — no Docker required.

## Run it

```sh
./tools/e2e/scripts/run.sh
```

One command. It builds the shared workspace image (`tools/e2e/Containerfile`, via
`scripts/build.sh`) and, on top of it, the harness's thin runtime layer
(`tools/e2e/Containerfile.e2e` — just the synthetic workload + entrypoint), brings up
Postgres + RustFS + the full DC stack (`podman compose`, `tools/e2e/compose.yaml`), and
runs the full sequence below. Every gate is a hard assertion — the script exits
non-zero (and, by default, tears the stack down after dumping logs to
`tools/e2e/.run/`) on any violation. Set `DC_E2E_KEEP=true` to leave a failed stack
running for interactive debugging instead.

`tools/e2e/Containerfile` (the workspace build) is the **same base image**
`.github/workflows/ci.yaml` uses — there's no separate CI-only build. CI runs
`colcon test` (`scripts/test.sh`) directly against that base image; it never builds
`Containerfile.e2e` at all, since the harness's synthetic workload/entrypoint are
irrelevant to running the test suite. Building and testing are both plain scripts a
developer runs the same way locally.

By default the induced outage is the PRD's full 10 minutes
(`DC_E2E_OUTAGE_SECONDS=600`); override it for faster local iteration, e.g.:

```sh
DC_E2E_OUTAGE_SECONDS=30 ./tools/e2e/scripts/run.sh
```

`.github/workflows/ci.yaml` runs this same script with a 60s outage window (same
code paths, shorter wall-clock) so the harness is a real, CI-gating check on every PR,
not just a local tool.

## What it does

1. **Reference workload** (`tools/e2e/params/e2e_params.yaml`): 20 Measurements at 1 Hz
   — 6 real, hardware-free `dc_measurements` plugins (`memory`, `os`, `storage`,
   `uptime`, `tcp_health`, `dummy`) plus 14 `StringStamped` Measurements fed by
   `tools/e2e/scripts/workload_generator.py`, a synthetic rclpy node that publishes a
   strictly-incrementing per-topic counter — the verification step below uses that
   counter to detect gaps (loss) and repeats (double delivery) independently of
   timestamp precision. Plus one `camera` Measurement, fed synthetic
   `sensor_msgs/msg/Image` frames every 15s by the same generator, driving the real
   File-upload pipeline (ADR-0005) end to end.
2. **Startup gate**: measures wall-clock from starting the `dc` container to the first
   Record landing in Postgres; hard-fails above 10s.
3. **Steady state**, then an **induced outage**: the harness stops the Postgres and
   RustFS containers (not a network-namespace disconnect) for the outage window.
   Operationally this is equivalent to a real network partition from dc_bridge/Vector's
   point of view — both surface as "destination unreachable," and both are recovered by
   the exact same disk-buffer-then-reconnect path (ADR-0002) — and it sidesteps
   rootless-Podman network-disconnect semantics that vary by version. `dc` (and the
   synthetic workload) keep running throughout, so Records keep being produced and
   buffered to disk while the destinations are down.
4. **Full stack restart** while destinations are still down (`podman compose restart
   dc`), then destinations are restored and the stack drains.
5. **Verification** (`tools/e2e/scripts/verify_zero_loss.py`): queries Postgres via
   `podman compose exec` (no host psycopg2/psql needed) and hard-fails on: any missing
   or duplicated synth-topic counter value, zero Records for any of the 20 measurement
   Tags, duplicate `(tag, date)` rows for the real measurements, or any camera File
   whose "uploaded" status row appears more than once (an Uploader idempotency
   violation, ADR-0005/#248). Nothing here is a skip-on-missing check — a table that
   doesn't exist or a query that fails is a FAIL, not silence.
6. **Resource usage** (`tools/e2e/scripts/measure_resources.sh`): samples the `dc`
   container's CPU/RSS throughout the run into `tools/e2e/.run/resource_usage.csv`.
   Informational only, per the PRD — it does not gate the run.

## Layout

- `Containerfile` — builds the full DC workspace (every `dc_*` package, all C++ since
  ADR-0007). Single stage: an apt/toolchain `RUN` (no DC source — rarely changes, so it
  stays a build-cache hit) followed by the `rosdep install`/`colcon build` `RUN` (`ARG
  CCOV` gates coverage-instrumented compiler flags). Jazzy CI (`ci.yaml`) builds and uses
  **only** this image — `colcon test` runs directly against it, no harness-specific files
  needed.
- `Containerfile.e2e` — a thin `FROM <workspace image>` layer adding just the harness's
  own runtime bits (the synthetic workload generator, its params, the entrypoint).
  Never built by CI. Its build context is `tools/e2e/` itself, not the repo root — it
  doesn't need `.dockerignore` handling at all (see below).
- `compose.yaml` — `postgres` (pre-seeded via `sql/init.sql` — Vector's `postgres` sink
  maps JSON keys onto existing columns and doesn't create them, see
  `dc_bridge/dc_bridge_core/tests/end_to_end.rs`), `rustfs`, and `dc`. Named volumes
  (not bind mounts) so `dc_bridge`'s disk buffer and Postgres's data directory survive
  a full stack restart — that persistence is what makes the zero-loss guarantee hold
  across a restart, not just a live process.
- `params/e2e_params.yaml` — the reference workload's ROS parameters (measurement
  plugins, `dc_bridge` destinations: `pgsql_records`, `pgsql_files` for the Uploader's
  metadata Records, `rustfs` for the actual File bytes).
- `scripts/workload_generator.py` — the synthetic counter/image publisher (runs inside
  the `dc` container, started by `scripts/entrypoint.sh` alongside the real DC stack).
- `scripts/build.sh` — builds `Containerfile` (the shared workspace image: handles the
  `.dockerignore` dance below, the coverage `ARG`, and an optional registry-backed
  `--cache-from`/`--cache-to`). Used by both `run.sh` and `ci.yaml`.
- `scripts/test.sh` — runs `colcon test` (C++ gtest + Rust cargo) plus coverage against
  an already-built (workspace) image. Also used by both a developer locally and CI —
  see `ci.yaml`'s `build-and-test` job.
- `scripts/run.sh` — the one-command harness orchestrator described above (calls
  `build.sh` for the workspace image, then builds `Containerfile.e2e` on top of it).
- `scripts/verify_zero_loss.py` — the hard-failing Postgres verification.
- `scripts/measure_resources.sh` — the informational resource sampler.

## `.dockerignore`

This repo's root `.dockerignore` is an "ignore everything, whitelist `package.xml`"
allowlist scoped to the legacy `docker/ci/Dockerfile`'s narrow rosdep-only needs (mirrors
`docker.yaml`'s own `source`/`source-sim` jobs, which `rm .dockerignore` before a
full-source build for the same reason). `scripts/build.sh` moves it aside for the
duration of the build and restores it afterward — `podman build --ignorefile` does
**not** override `.dockerignore`, only supplements it (verified empirically).

## Not done / left for follow-up

- The full 10-minute outage window is only exercised locally by default (CI uses 60s
  for turnaround time); a scheduled/nightly job running the full 10-minute window in CI
  would close that gap fully but wasn't set up as part of this issue.
