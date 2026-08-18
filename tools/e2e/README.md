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

The images are the **same artifacts CI builds**, built the **same way** — there's no
CI-only build path. In CI each image is built and pushed by its own job, and `run.sh`
is handed the prebuilt `dc-e2e` image via `DC_E2E_IMAGE` so it runs that exact artifact
instead of rebuilding (`DC_WORKSPACE_IMAGE` similarly reuses a prebuilt workspace
base). `build.sh` / `test.sh` / `run.sh` are the same plain scripts a developer runs
locally, and CI's `build-workspace` job (`ci.yaml`) calls `build.sh` directly — it adds
two things a local build doesn't need: `CACHE_REF`, a registry ref `build.sh` passes to
`podman build --cache-from/--cache-to` so a cold GitHub-hosted runner still starts from
warm *layers* (the rarely-changing apt/toolchain/aws_sdk_vendor ones) rather than
podman's otherwise local-only layer cache; and `BUILDAH_TMPDIR`, which relocates the
`workspace` stage's `ccache` mount (see `Containerfile` below) into a directory
`actions/cache` persists between runs — that mount lives entirely outside
`--cache-from`/`--cache-to`'s reach (verified: it carries only layers, never
cache-mount content).

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
6. **Passthrough coverage** (ADR-0003, `params/e2e_passthrough_sink.toml`): a raw Vector
   `file` sink loaded through `custom_config_files` — a sink `dc_bridge` has no knowledge
   of — consuming the public `dc.<tag>` routes for `synth00`/`synth01` alongside the
   blessed Destinations. Before this, the passthrough was covered only by
   `render_test.cpp`'s pure `validate_custom_config_files()` unit tests (TOML parsing,
   component-id collisions); nothing asserted that a passthrough sink actually *receives*
   Records. A `file` sink rather than a networked one on purpose: it needs no extra
   container, and it stays up through the induced outage, so a gap in its output is
   unambiguously the pipeline losing a Record rather than a store that hadn't come back.
7. **MCAP passthrough coverage** (ADR-0009, `params/e2e_mcap_sink.toml`, #210): a second,
   differently-shaped passthrough alongside the one above — Vector has no MCAP sink at
   all, so this is a `socket` sink streaming `synth02`/`synth03` to the standalone
   `dc_mcap_writer` process (started by `entrypoint.sh` before the DC stack), which
   writes them to `.mcap` files under the `dc_e2e_data` volume. Held to the same
   zero-loss/only-subscribed-Tags standard as the NDJSON passthrough, via a JSON summary
   of the capture (`scripts/mcap_summary.py`, which needs the `mcap` library the DC image
   installs — not this script's own host runner).
8. **Raw / generic-subscription coverage** (#227, the `raw:` block in
   `params/e2e_params.yaml`): `dc_bridge` also subscribes *generically* to
   `/dc/e2e/synth/synth00` — the workload generator's own source topic, carrying
   `dc_interfaces/msg/StringStamped`, a custom non-`std_msgs` type the Bridge has no
   compile-time knowledge of — and ships it to its own `file` Destination under the
   `dc.raw.` Tag namespace, beside the Measurement path that same topic already feeds.
   `check_raw()` asserts the Records arrived, decoded field-for-field (both strings plus
   the nested `Header`/`Time`, proving the runtime introspection walk is complete),
   carry no Tag from outside the namespace (the prefix route branch discriminates), and
   have no gaps *inside the window raw mode was subscribed for* — unlike a Measurement, a
   raw subscription doesn't exist until the topic is discovered, so values published
   before the first discovery were never offered to the pipeline at all.
9. **Verification** (`tools/e2e/scripts/verify_zero_loss.py`): queries Postgres via
   `podman exec` on the Postgres container (no host psycopg2/psql needed). The shipper is
   at-least-once (ADR-0002), so it **hard-fails on loss**. Loss is found by diffing what
   arrived against `params/../workload_ledger.txt` — the generator's own record of every
   value it published, written to the `dc_e2e_data` volume so it survives the restart.
   That comparison is one-to-one against an independent source. It replaced
   `expected = max(value) + 1`, which read its expectation out of the table it was
   checking and so could not see a missing tail: deliver a clean 0..84 having lost 85..96
   and `max` drops to 84, the bar drops with it, and the check passed with 12 Records gone
   (#312). A **duplicate** is reported as a note and deduplicated on read.

   Two gaps are tolerated rather than failed, both bounded by `MAX_KILL_GAP` and both
   reported: the tick in flight when the harness restarts the DC container, and the last
   Records at shutdown. The workload generator runs *inside* that container, so a message
   it published as the process died was never handed to `measurement_server` and no buffer
   could have held it. A gap anywhere other than a kill point is loss and fails.

   Nothing here is a skip-on-missing check — a table that doesn't exist or a query that
   fails is a FAIL, not silence. The passthrough sink's own output (`check_passthrough`)
   is held to the same standard and on the same ledger basis: every published Record must
   have reached it, it must have received *only* the Tags it subscribed to (proving the
   per-Tag route branches discriminate rather than fanning everything out), and a missing
   or empty output file is a FAIL, never a skip.
10. **Resource usage** (`tools/e2e/scripts/measure_resources.sh`): samples the `dc`
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

## Incident-capture scenario (#291)

A third narrow harness, for the `incident_id` field an armed Measurement stamps on the
pre-event window it releases:

```sh
./tools/e2e/scripts/run_incident.sh
```

Same `dc-e2e` image and env vars again, this time against
`params/e2e_incident_params.yaml`: an `uptime` Measurement armed with
`buffer_duration_sec` beside a `memory` Measurement collecting live, both routed to one
`postgres` Destination. The scenario publishes a single `FlushEvent` onto `/dc/flush`
with a known id and asserts: (1) `dc_records.incident_id` exists as a real `text`
column; (2) while armed, the buffered Measurement ships *nothing* while the live one
keeps landing rows; (3) after the event, the released window is reachable as `WHERE
incident_id = '…'` — a column predicate, which is the whole point of #291, since the
same data buried in the JSON payload would not be; (4) the id is on the released window
only, and the never-armed Measurement's rows keep `incident_id IS NULL`.

The `FlushEvent` is published with `ros2 topic pub` rather than by a `dc_triggers`
broadcast node: that topic is the contract Measurements subscribe to, `dc_triggers` has
its own tests for minting the event, and the broadcast node is not in `dc_bringup`'s
launch yet. Not run by `ci.yaml`, same as the retention scenario.

## Degraded-network scenario (#366)

A fourth scenario, proving the zero-loss guarantee above holds under stated network
conditions instead of only over intra-container loopback:

```sh
./tools/e2e/scripts/run_degraded.sh                         # loopback profile (baseline)
DC_E2E_PROFILE=poor-wifi ./tools/e2e/scripts/run_degraded.sh
```

Same `dc-e2e` image and reference workload as `run.sh` (`params/e2e_degraded_params.yaml`
is byte-for-byte `params/e2e_params.yaml` except for the two Destination hosts — see that
file's header), but Postgres and RustFS run on their own bridge network instead of
`--network host`, so the path to them can be shaped without shaping the host itself. A
named profile (`scripts/network_profiles.py`: `loopback`, `good-wifi`, `poor-wifi`,
`site-uplink` — one declarative place, so two runs of the same name are comparable) is
applied as a `tc netem` qdisc on Postgres's and RustFS's own interfaces, via a short-lived
helper container that joins their network namespace (`--network container:<name>
--cap-add=NET_ADMIN`) rather than granting either image any capability itself. A shaping
pre-check measures the actual TCP connect-time round trip before any workload runs
(`scripts/measure_rtt.py`) and hard-fails the run if the measurement isn't consistent with
the requested profile — a degraded run must never pass silently as loopback because
shaping failed to apply. Partway through the run, a LINK fault (`tc netem loss 100%` on
both Destinations, then restored) proves the network going away is handled distinctly from
`run.sh`'s container stop/restart — nothing here resets Forwarder state. The network
conditions a run used, plus the measured pre-check round trip, are written to
`tools/e2e/.run/conditions.json` and folded into `verify_zero_loss.py`'s report — a result
is never separable from the conditions that produced it.

Not wired into `ci.yaml`, same as the retention and incident scenarios below: whether it
gates merges is left for once its runtime and stability are measured (#366's own "Out of
Scope"). The Forwarder's own unacked-window-depth, backpressure-classification, and
resend-under-delay behaviour under acknowledgement delay is covered separately and much
more cheaply at `dc_bridge/test/forwarder_test.cpp`'s existing mock-ingest-peer seam (an
`AckPolicy` describing when/whether a chunk gets acknowledged) — no container needed for
that; see that file for the corresponding unit tests.

## Layout

- `Containerfile` — builds the full DC workspace (every `dc_*` package, all C++ since
  ADR-0007), in three stages: `cacher` (extracts just the `package.xml` manifests from
  the full source tree, so `toolchain`'s own dependency-install layer keys its cache on
  dependency lists, not on every source edit, without hardcoding one `COPY` per
  package), `toolchain` (apt/rosdep + a full-workspace rosdep install off those
  manifests + pre-built `aws_sdk_vendor` — rarely changes, so it stays a build-cache
  hit), and `workspace` (inherits `toolchain`, `COPY`s full source, runs `rosdep
  install`/`colcon build`/`colcon test`; `ARG CCOV` gates coverage-instrumented
  compiler flags). The `workspace` stage's compile/test step uses a
  `RUN --mount=type=cache` ccache mount, which persists independently of that RUN's own
  layer (so it survives even though the layer itself invalidates on every source
  change) — but only if `$TMPDIR` points somewhere that itself persists between
  builds, since buildah stores the mount under `$TMPDIR/buildah-cache/<id>`, entirely
  outside `--cache-from`/`--cache-to`'s reach (verified: it round-trips only layers
  through the registry, never cache-mount content). CI arranges that persistence via
  `scripts/build.sh`'s `BUILDAH_TMPDIR` + an `actions/cache` step in `ci.yaml` — so a
  change to one file recompiles that file's translation units, not the whole
  workspace, on a fresh runner too. `colcon test`
  failing doesn't fail the `podman build` itself (see the Containerfile's own
  comments) — it's recorded in the image's `/root/ws/TEST_RESULT` instead, so the image
  still gets tagged and its `coverage/cpp.info` is still extractable even when a test
  fails; `ci.yaml` checks `TEST_RESULT` after extracting coverage and only pushes the
  image when it reads `0`.
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
  metadata Records, `rustfs` for the actual File bytes, `raw_file` for raw mode's own
  output), plus the `raw:` block (#227) and the `custom_config_files` entry that loads the
  passthrough snippet below.
- `params/e2e_passthrough_sink.toml` — the ADR-0003 passthrough snippet: a raw Vector
  `file` sink on the `dc.<tag>` routes for `synth00`/`synth01`, written to the
  `dc_e2e_data` volume so it survives the mid-run restart. Note its `inputs` name topics
  that `pgsql_records` already lists, and must: `dc_bridge` builds its subscriptions and
  route branches from `destinations` alone and never reads a snippet's `inputs`, so a
  topic listed only in the snippet would resolve to a route that does not exist.
- `params/e2e_mcap_sink.toml` — the ADR-0009 MCAP passthrough snippet (#210): a Vector
  `socket` sink on the `dc.<tag>` routes for `synth02`/`synth03`, streamed to the
  standalone `dc_mcap_writer` process rather than to a Vector-native sink (Vector has no
  MCAP sink at all). Same `inputs`/`pgsql_records` constraint as the `file` sink above.
- `scripts/workload_generator.py` — the synthetic counter/image publisher (runs inside
  the `dc` container, started by `scripts/entrypoint.sh` alongside the real DC stack).
- `scripts/mcap_summary.py` — reads back `dc_mcap_writer`'s `.mcap` capture and emits a
  JSON summary `verify_zero_loss.py` can consume; runs inside the DC image (the only
  place the `mcap` library is installed), invoked by `run.sh` as a one-off container on
  the `dc_e2e_data` volume, the same pattern used to extract the workload ledger and the
  NDJSON passthrough's output.
- `scripts/build.sh` — builds `Containerfile` end to end (handles the `.dockerignore`
  dance below, the coverage `ARG`, `--network host` so the `workspace` stage's `colcon
  test` can reach the test-dependency stores, an optional registry-backed
  `--cache-from`/`--cache-to` for layers, and an optional `BUILDAH_TMPDIR` to relocate
  the ccache `RUN --mount=type=cache` mount somewhere the caller can persist across
  runs). The one build path for local dev and CI alike; set `TARGET=toolchain` to
  build only the `toolchain` stage instead, for debugging.
- `scripts/test.sh` — runs `colcon test` (C++ gtest) plus coverage against an
  already-built workspace image, for a developer who wants to re-run tests without a
  full rebuild. CI doesn't call this — `colcon test` is part of the `workspace` stage's
  own build step (see `Containerfile` above).
- `scripts/run.sh` — the one-command harness orchestrator described above. Builds the
  workspace + `dc-e2e` images locally, or runs a prebuilt `dc-e2e` image when handed one
  via `DC_E2E_IMAGE` (as CI's `e2e` job does).
- `scripts/verify_zero_loss.py` — the hard-failing Postgres + passthrough verification.
- `scripts/measure_resources.sh` — the informational resource sampler.
- `params/e2e_retention_params.yaml` / `scripts/run_retention.sh` — the files retention
  scenario (#267) described above; a separate, narrower harness reusing the same
  `dc-e2e` image.
- `params/e2e_incident_params.yaml` / `scripts/run_incident.sh` — the incident-capture
  scenario (#291) described above, likewise reusing the same `dc-e2e` image.
- `params/e2e_degraded_params.yaml` / `scripts/run_degraded.sh` — the degraded-network
  scenario (#366) described above; a fourth sibling harness, reusing the same `dc-e2e`
  image and `verify_zero_loss.py`, but its own bridge network so the path to the
  Destinations can be shaped.
- `scripts/network_profiles.py` — the named network conditions (#366) `run_degraded.sh`
  applies, in one declarative place also meant for #323's limits harness to reuse.
- `scripts/measure_rtt.py` — stdlib TCP connect-time sampler (#366), used both for
  Destination readiness polling and the shaping pre-check `run_degraded.sh` never skips.

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
