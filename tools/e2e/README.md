# Zero-loss E2E harness (#249)

Proves the DC 2.0 (Jazzy) pipeline's reliability claims end-to-end: launch-to-first-
Record under 10 seconds, and zero Record/File loss across a destination outage plus a
full stack restart. Runs with **Podman** (see `CLAUDE.md` "Containers: Podman, not
Docker") — no Docker required.

## Run it

```sh
./tools/e2e/scripts/run.sh
```

One command. Locally it builds the shared workspace image (`tools/e2e/Containerfile`,
via `scripts/build.sh`) and, on top of it, the harness's thin runtime layer
(`tools/e2e/Containerfile.e2e` — just the synthetic workload + entrypoint), then brings
up Postgres + RustFS + the full DC stack with plain `podman` (no compose — the
outage/restart lifecycle below *is* the test, and driving the containers by hand is
simpler and needs nothing installed) and runs the full sequence below. Every gate is a
hard assertion — the script exits non-zero (and, by default, tears the stack down after
dumping the `dc` container log to `tools/e2e/.run/`) on any violation. Set
`DC_E2E_KEEP=true` to leave a failed stack running for interactive debugging instead.

The images are the **same artifacts CI builds** — there's no CI-only build path. In CI
each image is built and pushed by its own job, and `run.sh` is handed the prebuilt
`dc-e2e` image via `DC_E2E_IMAGE` so it runs that exact artifact instead of rebuilding
(`DC_WORKSPACE_IMAGE` similarly reuses a prebuilt workspace base). `build.sh` /
`test.sh` / `run.sh` are the same plain scripts a developer runs locally.

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
4. **Full stack restart** while destinations are still down (`podman restart` on the
   `dc` container), then destinations are restored and the stack drains. This is this
   harness's stand-in for #265's own acceptance criterion (a files-Destination Record
   published with the store down, the Bridge process restarted, the store then
   restored) — the camera Measurement's File goes through exactly that path every run.
5. **Durable upload intent queue check** (#265): asserts the on-disk intent queue
   (`dc_bridge`'s `<shipper.data_dir>/queue/upload/`, on the `dc_e2e_buffer` named
   volume) is empty once the run stops — proof the queue actually drained rather than
   silently orphaning a pending upload across the restart above (the pre-#265 in-memory
   queue would have forgotten it).
6. **Verification** (`tools/e2e/scripts/verify_zero_loss.py`): queries Postgres via
   `podman exec` on the Postgres container (no host psycopg2/psql needed). The shipper is
   at-least-once (ADR-0002), so it **hard-fails on loss** — a gap in a synth topic's
   counter (checked against the *distinct* values, so a re-send can't hide a gap), or
   zero Records for any of the 20 measurement Tags. Since #309 it **also hard-fails on
   duplicates**: re-delivery still happens, but the destination schema collapses it on
   write (a `BEFORE INSERT` trigger keyed on `(tag, date)` — see `sql/init.sql`), so a
   duplicate row reaching the verifier means that dedup did not work. `dc_files` is the
   one exception and still reports duplicates as notes: its rows are timestamped at emit
   time, so `(tag, date)` is not their identity. Because a re-delivery is not
   deterministic (a quiet run may produce none), the verifier also **injects a known
   duplicate** and asserts the trigger skips it — writing it in one statement alongside
   two fresh rows, so a UNIQUE constraint (which would abort the whole batch) fails the
   check with its own message rather than passing. Nothing here is a skip-on-missing check — a table
   that doesn't exist or a query that fails is a FAIL, not silence.
7. **Resource usage** (`tools/e2e/scripts/measure_resources.sh`): samples the `dc`
   container's CPU/RSS throughout the run into `tools/e2e/.run/resource_usage.csv`.
   Informational only, per the PRD — it does not gate the run.

## Files retention scenario (#267)

A second, narrower harness for the retention policy — separate from `run.sh` above
because it needs the opposite starting condition (RustFS *never* comes up until told
to, rather than an outage induced mid-run) and a dedicated small
`files.retention.max_bytes`:

```sh
./tools/e2e/scripts/run_retention.sh
```

Reuses the same `dc-e2e` image (same `DC_E2E_IMAGE`/`DC_WORKSPACE_IMAGE` env vars as
`run.sh`) but overrides the mounted params file with
`params/e2e_retention_params.yaml` — RustFS starts down and stays down while the camera
Measurement's captures pile up in the durable upload intent queue, past a deliberately
small `max_bytes`. Asserts: (1) with RustFS still down, a `deleted: true, uploaded:
false` audit row lands in `dc_files` for the oldest capture, and its local File is
actually gone from disk (File+intent atomicity); (2) once RustFS comes up, a later
(unshed) capture uploads and verifies normally. Not run by `ci.yaml` yet — `run.sh`'s
own camera path already covers the File pipeline's steady-state/outage behavior; wiring
this scenario into CI as its own job is left as a follow-up, same as the "not done" list
below.

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
- `sql/init.sql` — pre-creates the `dc_records` / `dc_files` tables (Vector's `postgres`
  sink maps JSON keys onto existing columns and doesn't create them). `run.sh` runs
  `postgres`, `rustfs`, and the `dc` stack as plain `podman` containers on **named
  volumes** (not bind mounts) so `dc_bridge`'s disk buffer and Postgres's data directory
  survive a full stack restart — that persistence is what makes the zero-loss guarantee
  hold across a restart, not just a live process.
- `params/e2e_params.yaml` — the reference workload's ROS parameters (measurement
  plugins, `dc_bridge` destinations: `pgsql_records`, `pgsql_files` for the Uploader's
  metadata Records, `rustfs` for the actual File bytes).
- `scripts/workload_generator.py` — the synthetic counter/image publisher (runs inside
  the `dc` container, started by `scripts/entrypoint.sh` alongside the real DC stack).
- `scripts/build.sh` — builds `Containerfile` (the shared workspace image: handles the
  `.dockerignore` dance below, the coverage `ARG`, and an optional registry-backed
  `--cache-from`/`--cache-to`). Used by both `run.sh` and `ci.yaml`.
- `scripts/test.sh` — runs `colcon test` (C++ gtest) plus coverage against an
  already-built workspace image. Used by both a developer locally and CI's `colcon-test`
  job.
- `scripts/run.sh` — the one-command harness orchestrator described above. Builds the
  workspace + `dc-e2e` images locally, or runs a prebuilt `dc-e2e` image when handed one
  via `DC_E2E_IMAGE` (as CI's `e2e` job does).
- `scripts/verify_zero_loss.py` — the hard-failing Postgres verification.
- `scripts/measure_resources.sh` — the informational resource sampler.
- `params/e2e_retention_params.yaml` / `scripts/run_retention.sh` — the files retention
  scenario (#267) described above; a separate, narrower harness reusing the same
  `dc-e2e` image.

## `.dockerignore`

This repo's root `.dockerignore` is an "ignore everything, whitelist `package.xml`"
allowlist, originally scoped to a narrow rosdep-only image (the now-removed
humble-line `docker/ci/Dockerfile`). `scripts/build.sh` moves it aside for the
duration of this build, which needs every source file, and restores it afterward —
`podman build --ignorefile` does **not** override `.dockerignore`, only supplements it
(verified empirically).

## Not done / left for follow-up

- The full 10-minute outage window is only exercised locally by default (CI uses 60s
  for turnaround time); a scheduled/nightly job running the full 10-minute window in CI
  would close that gap fully but wasn't set up as part of this issue.
