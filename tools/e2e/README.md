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

## Synthetic load driver (#378)

The first piece of #323's limits harness: `scripts/load_driver.py` speaks the shipper
ingest protocol directly — the same msgpack-over-TCP Forward Mode frames, chunk id and
all, that `dc_bridge`'s Forwarder emits (see `dc_bridge/src/forwarder.cpp`) — closely
enough that a Shipper's `fluent` ingest listener cannot tell it from a real Bridge at the
socket. Given a target, a connection count, a per-connection Record rate, and a duration,
it opens that many concurrent TCP connections, sends frames at that rate on each, and
returns a sent-ledger plus per-frame acknowledgement-latency observations:

```sh
uv run tools/e2e/scripts/load_driver.py <host> <port> --connections 200 --rate 5 --duration 30
```

It has no ROS dependency and no DC stack dependency — it is what lets the eventual limits
harness represent hundreds of robots on one developer machine alongside a handful of real
DC stacks (see #323's PRD). `tools/e2e/test/test_load_driver.py` covers its wire format
(pinned against `forwarder_test.cpp`'s own EventTime bytes) and its rate/ack/ledger
behaviour against an in-process fake fluent listener, standalone, no container needed.
Byte-compatibility with a *real* Shipper build is a separate, container-based claim a fake
can't establish:

```sh
./tools/e2e/scripts/run_load_driver_shipper_test.sh
```

Pulls the exact Vector version `vector_vendor/CMakeLists.txt` pins, runs it as a bare
`fluent` source (global acknowledgements on, exactly as `render.cpp` configures it for
the real Bridge) plus a `file` sink, points the driver at it, and asserts every frame was
accepted and acknowledged and that the driver's ledger names the exact same records
Vector actually decoded — not just a matching count. Not wired into `ci.yaml`, same as
the retention/incident/degraded scenarios above.

The saturation probe (`scripts/saturation_probe.py`, #377) is a third piece landed
alongside it: a pure function over a window of ack-latency/unacked-window-depth/disk-
buffer observations, checked in the PRD's stated priority order and unit-tested alone.

The ramp controller (`scripts/ramp_controller.py`, #379) is the fourth. Given a driver, a
probe, and a `RampPolicy` of ascending load levels, `find_knee()` drives each level in
turn and asks the probe whether it saturated, stopping at the first tripped level and
reporting the knee: the highest level whose verdict stayed clear, plus the level that
tripped it. A policy whose every level stays clear is reported as `BOUND_NOT_FOUND`
rather than a fabricated ceiling at the top of the ramp — the failure mode #323's PRD
calls out as mattering most. The controller touches no I/O and knows nothing about what a
"level" means (a connection count, a rate, an upload concurrency, …); `driver` and `probe`
are plain callables, so `test_ramp_controller.py` covers it entirely with in-process
fakes, including the ramp-policy boundary cases (first step, last step, a single-step
ramp).

The curve reporter (`scripts/curve_reporter.py`, #380) is the fifth. It turns a set of
per-axis runs — each an ascending sequence of levels tried, whether each level was
actually driven and measured or is a projected `EXTRAPOLATED` point, its real-vs-
synthetic composition, and a CPU/memory/disk `ResourceSample` where one was taken — into
a stable, diffable `CurveReport` (`build_report()`) plus a human-readable summary
(`render_summary()`). It is a pure function, decoupled from `ramp_controller`/
`saturation_probe` the same way those two are decoupled from each other: no I/O, no
containers, its own `RunOutcome` mirroring `RampOutcome`'s values. `build_report()` sorts
axes by name and each axis's points by level so the same set of runs always yields the
same report regardless of collection order — required for a diff between two runs to
show only a real change. The binding constraint at saturation (CPU, memory, or disk) is
the resource that peaked highest across a run's *measured* points only (CPU-then-memory-
then-disk as the deterministic tie-break), and is reported as unavailable rather than
guessed when the axis never saturated or its knee has no accompanying resource sample. An
extrapolated point never carries a resource sample in either the structured report or the
summary — labelling every point `measured`/`extrapolated` in both is how the reporter
avoids presenting a projection as a measurement, per #323's PRD.
`tools/e2e/test/test_curve_reporter.py` covers all of this against synthetic run data
alone.

## Two-tier topology composer (#381)

The sixth piece of #323's limits harness: brings up, with plain podman on a dedicated
bridge network, the topology #323's PRD is ultimately sized around — a private site where
robots have no direct path to the cloud, and an edge server aggregates their Records and
forwards them upstream:

```sh
./tools/e2e/scripts/run_limits_two_tier.sh
```

A shared Postgres + RustFS (the real Destinations), an aggregating Shipper (a standalone
Vector instance, `type = "vector"` + `type = "fluent"` sources, no dc_bridge/ROS
involved), N real DC stacks (`params/e2e_limits_params.yaml`, each its own dc_bringup +
dc_bridge + local Shipper), and M synthetic senders (one `scripts/load_driver.py`
invocation, #378). At a fixed, non-saturating load, it reuses `verify_zero_loss.py`
**unmodified** to prove Records survive the extra hop.

Each real stack's own local Shipper has no Destination that writes to the real Postgres
at all — `params/e2e_limits_forward_sink.toml` (an ADR-0003 `custom_config_files`
passthrough, the same mechanism `e2e_passthrough_sink.toml`/`e2e_mcap_sink.toml` already
use) wires a `type = "vector"` sink to the same `dc.<tag>` route branches a blessed
Destination would, relaying them over the network to the aggregating Shipper via Vector's
own native inter-instance protocol — the standard way to chain a local ("agent") Vector to
a central ("aggregator") one. The aggregating Shipper's own `postgres` sink is the only
thing that ever writes Measurement/synth Records into the real `dc_records` table. The M
synthetic senders hit the aggregating Shipper's `fluent` source directly, adding the
realistic connection count and byte rate #323's PRD calls for; their frames aren't
`dc_records`-shaped, so they land on a separate `blackhole` sink and are verified via the
driver's own sent/acked ledger instead of a Postgres row check.

Two discoveries came out of implementing this, both recorded rather than worked around:

1. **Files bypass the two-tier chain.** The Uploader — File bytes over its own AWS SDK
   client, plus its own file-metadata Records — talks directly to whatever `s3`/
   `postgres` Destination host is configured, with no Vector hop at all
   (`dc_bridge/src/bridge_node.cpp`'s `run_uploader_worker` never touches the rendered
   Vector config). There is no way to route Files through the aggregating Shipper without
   new DC code, so `e2e_limits_params.yaml` points the Files-side Destinations straight at
   the real shared Postgres/RustFS. Extending true two-tier coverage to Files is left as a
   follow-up.
2. **The aggregating Shipper's one `postgres` sink is a shared bottleneck N real stacks
   don't have in any single-tier scenario** (each of those writes straight to its own
   Postgres connection). Verified empirically: two real stacks reliably produced far more
   at-least-once re-delivery than one does — past what `verify_zero_loss.py`'s
   zero-tolerance memory-timestamp-uniqueness check accepts, even though every value still
   arrived (the "duplicates" are identical `(date, value)` pairs, confirmed against
   Postgres directly — genuine at-least-once repeats, not loss). `DC_E2E_LIMITS_REAL_STACKS`
   stays configurable rather than hardcoded — finding exactly where that bottleneck sits is
   the ramp controller's job (#323's next piece), not this composer's — but its default is
   1 real stack so this script passes reliably on its own, matching #381's own scope
   ("before any ramping logic exists, at a fixed, non-saturating load").

Not wired into `ci.yaml`, same as the retention/incident/degraded/load-driver scenarios
above.

## Single-robot ceiling axis (#383)

The seventh piece: ramps one real DC stack's per-topic Record emission rate — through
real Measurements, the real Bridge, and the stack's own local Shipper, rather than
#378's socket-level synthetic sender — until the saturation probe trips, reporting the
maximum sustained per-robot Record rate:

```sh
./tools/e2e/scripts/run_limits_single_robot_ceiling.sh
```

Brings up one real DC stack plus Postgres/RustFS (the same single-stack topology
`run.sh` uses), then hands off to `scripts/run_limits_single_robot_ceiling.py`, which
wires this axis's own driver adapter (`scripts/single_robot_ceiling_driver.py`) to the
**unmodified** `ramp_controller.find_knee()`/`saturation_probe.evaluate()` — per #323's
PRD, "the ramp controller and saturation probe are reused unchanged; only the driver
adapter and topology wiring are new". The driver conforms to the same target/rate/
duration → ledger-plus-latency interface `load_driver.py` implements, but gets there by
polling the real stack instead of holding open a raw TCP connection: `set_rate()` steps
`workload_generator.py`'s `rate_hz` parameter live via `ros2 param set` (the node now
carries a parameter callback that recreates its publish timer on change, so a ramp step
doesn't require restarting the container), and each poll compares the ledger-checked
synth topic's published counter values against what has reached Postgres — the same
natural key `verify_zero_loss.py` already treats as ground truth — plus the local
Shipper's on-disk buffer size, direct evidence for the probe's disk-buffer-growth
signal. Because there is no per-frame ack visible from outside `dc_bridge`'s own
Forwarder/Shipper connection, the reported ack latency is a poll-resolution upper bound
on the real end-to-end delay, not the exact per-record figure `load_driver.py`'s live
TCP session gets — a discovery recorded in `single_robot_ceiling_driver.py`'s own
docstring rather than presented as more precise than it is. The ramp result is folded
into `curve_reporter.py`'s `AxisRun`/`CurveReport`, the same structured report every
other axis in the epic produces.

Zero loss is asserted once, cumulatively, after the whole ramp finishes and a drain
window lets any backlog from the final (possibly saturating) level land, reusing
`verify_zero_loss.py` **unmodified** — not after every individual clear level, which
would mean stopping and restarting the stack (losing its buffer/ROS state) between
every ramp step and defeating the point of a continuous ramp. Every sustainable level's
traffic is a strict subset of what that one cumulative, post-drain check covers, so a
pass proves zero loss at each of them individually.

`tools/e2e/test/test_single_robot_ceiling_driver.py` covers the driver adapter with a
fake `RealStackHandle` and a virtual clock — no podman, no ROS, no real time — including
an end-to-end run through the unmodified `find_knee()`.
`tools/e2e/test/test_run_limits_single_robot_ceiling.py` covers the orchestrator's pure
parsing helpers and its find_knee-result-to-`AxisRun` folding logic; per #323's PRD
("the topology composer and the scenario script are not unit-tested; running them is
the test"), the real-I/O `PodmanRealStackHandle` itself and the shell script are
exercised by actually running the scenario, not unit tests. Not wired into `ci.yaml`,
same as the other scenario scripts above.

## Shipper fan-in axis (#382)

The eighth piece, and the epic's flagship axis: how many robots does one aggregating
Shipper serve before it applies backpressure?

```sh
./tools/e2e/scripts/run_limits_shipper_fanin.sh
```

A sibling of `run_limits_two_tier.sh` — its own gates, not a mode bolted onto the
zero-loss run — but it reuses that ticket's topology verbatim, since the two-tier chain
it composes is exactly what this ramp drives: the same `params/e2e_limits_params.yaml`,
`params/e2e_limits_forward_sink.toml`, and the same hard-coded container names those
files address (`dc-e2e-limits-postgres`/`rustfs`/`agg`). `scripts/run_limits_shipper_fanin.py`
is where the pieces actually meet: `ramp_controller.find_knee()` (#379) drives synthetic
connection count upward via `load_driver.py` (#378) against the aggregating Shipper,
`saturation_probe.evaluate()` (#377) judges each level from a short window of
ack-latency/unacked-window-depth/disk-buffer-growth observations, and `curve_reporter`
(#380) turns the result into a stable report plus the PRD's required closing sentence.
None of the four are modified — this axis is exactly the "one axis costs one thin
scenario" the epic's PRD called for.

Zero loss is asserted by hooking straight into the ramp controller's own driver/probe
seam: the `probe` handed to `find_knee()` first asks the saturation probe for a verdict,
and only when a level's verdict stays clear does it also run `verify_zero_loss.py` — "the
existing verifier," reused unmodified, same as `run_limits_two_tier.sh` — against a
snapshot of the ledger-checked real stack's still-running output. A violation there is a
hard failure of the whole ramp, immediately: a throughput figure obtained from a
configuration that was dropping Records is never reported. Recorded discovery, verified
against a real run: because the real stack keeps running (and its output files keep
growing) across the whole ramp rather than being restarted between every level, the
snapshot the verifier reads occasionally races that still-running container. One instance
of that race has a precise, diagnosed cause — `entrypoint.sh` runs `dc_mcap_writer` with
`--max-duration-secs 15`, and a `.mcap` file only becomes readable once its rotation
finishes, so a snapshot taken mid-rotation can report the handful of most-recent Records
as "never reached dc_mcap_writer" even though they are safely on disk. The check retries
any failure shape once, after a pause comfortably longer than that 15s rotation window,
before treating it as a hard failure like any other — never silently skipped, never given
more than one extra chance.

The scenario then proves the instrument itself, per the PRD's "a limits harness that
cannot demonstrate detecting an induced limit has not been shown to work": having found
the unconstrained ceiling, it truncates `dc_records`/`dc_files`, tears down that phase's
aggregating Shipper and real stacks (Postgres/RustFS/the network stay up), and re-runs the
identical ramp against a deliberately CPU/memory-constrained aggregating Shipper
(`--cpus`/`--memory`, `DC_E2E_FANIN_CONSTRAINED_CPUS`/`_MEMORY`). The constrained run's
ceiling must come in at or below half the unconstrained one
(`DC_E2E_FANIN_CEILING_DROP_RATIO`, default `0.5`) — a hard gate, not informational.
Either phase reporting `BOUND_NOT_FOUND` (never saturating within the tested levels) is
itself a hard failure: an unconstrained run with no measured ceiling has nothing for the
PRD's closing sentence to state, and a constrained run that never saturates has failed to
demonstrate the induced limit, never fabricated as a pass either way.

Not wired into `ci.yaml`, same as the other scenario scripts above — and, like
`run_limits_two_tier.sh`'s own dependents, this axis needs that script's own topology
bring-up already established working before it can run at all.

## Uploader concurrency axis (#384)

The ninth piece of #323's limits harness (developed in parallel with #383/#382, which
merged first and already call `ramp_controller.find_knee()` end to end for their own
axes): ramps the number of concurrent Bridge/Uploader processes against one shared
object-storage Destination until verify-then-delete falls behind, reporting the last
sustainable concurrency level.

```sh
./tools/e2e/scripts/run_limits_upload_concurrency.sh
```

The Uploader has no internal concurrency knob — `dc_bridge/src/uploader/uploader.hpp`
says so directly ("Synchronous (aws-sdk-cpp is blocking) — no async runtime"), one
`uploader_thread_` per Bridge process, and `bridge_node.cpp`'s full param schema declares
none. "Concurrency" on this axis therefore means starting N separate Bridge/Uploader
containers against a shared Postgres + RustFS on a dedicated bridge network — the same
topology #381 already proved works for Files, whose header recorded that Files bypass
that scenario's two-tier Shipper-relay chain entirely (no Vector hop at all) and left true
two-tier File coverage as a follow-up. This axis is the single-tier concurrency-and-custody
scenario #381 punted, not that follow-up.

`scripts/run_upload_concurrency_axis.py` owns the ramp itself: it calls
`ramp_controller.find_knee()` **unchanged**, driving load by starting/stopping N
containers per level against `params/e2e_limits_upload_params.yaml` (a minimal params
file carrying only the camera Measurement — this axis's claim is File concurrency and
custody, not Record delivery), and feeds the result to `curve_reporter.build_report()` in
its stable format. Per #384's acceptance criteria, file size stays the same deliberately
synthetic 64x64 solid-color image every E2E scenario already produces (~12KB via
`workload_generator.py`); only capture *cadence* is sped up per container
(`DC_E2E_CAMERA_PERIOD_S`, a new env-var override on `workload_generator.py` — default
15s is unchanged for every other scenario) so N processes produce enough uploads inside a
short steady-state window to show a trend at all. Any byte-volume implication of a
reported ceiling is arithmetic, never presented as measured.

Since Files never touch the Shipper, `saturation_probe.py`'s ack-latency/unacked-window/
disk-buffer signals don't apply here — `scripts/upload_saturation_probe.py` is this
axis's own pure verdict function (unit-tested alone, no container, same structure and
priority-ordering/outage-suppression discipline as its sibling) over two observables that
exist for Files instead: the Bridge's own `~/ready` service, which already exposes its
upload queue depth (#265's existing "cheap observability hook" — no DC code changed to
add it) trending upward as the primary signal, and the verify-then-delete backlog —
files verified in `dc_files` (ADR-0005) with no later delete row for the same
`local_path` yet — growing at steady state (suppressed during a declared outage) as the
secondary one. `dc_files` is an append-only event log, not a current-state table (Vector's
`postgres` sink only ever `INSERT`s), so computing that backlog correctly means "verified
with no *later* delete row exists", not "the verify row itself still says
`deleted=false`" — the first version of this script got that wrong and reported a
backlog that only ever grew, on a run that was actually healthy throughout; caught by
running the script for real against a live stack, not by reading the code.

At every level actually driven, two hard gates run against `dc_files` before that level's
containers are torn down, never folded into the saturation verdict since they are
correctness properties, not backpressure signals: no `deleted=true` row for a
`local_path` may exist without an earlier-or-equal `uploaded=true, deleted=false` row for
the same path (delete-after-verify ordering, checked empirically across N concurrently-
uploading processes writing to the shared table, not just assumed from one process's own
in-order code path), and every `group_complete` marker for the camera group names the
expected file count. A violation raises immediately and aborts the whole ramp.

Not wired into `ci.yaml`, same as every other narrow scenario above.

## Drain-rate axis (#385)

The tenth piece of #323's limits harness (developed in parallel with #382/#383/#384):
"how long does a backlog take to clear once connectivity returns, as a function of load
and outage length."

```sh
./tools/e2e/scripts/run_limits_drain_rate.sh
```

Brings up a standalone Shipper under test (a bare Vector instance on its own bridge
network, `fluent` source + `aws_s3` sink with a disk buffer — same convention the
two-tier composer's aggregating Shipper and the load-driver byte-compatibility test both
use) plus RustFS as the Destination it can lose, then drives `scripts/drain_rate_axis.py`
through an ascending ramp of outage lengths at a stated synthetic connection
count/rate. For each outage length it induces the outage with the exact `podman stop` /
wait / `podman start` recipe `run.sh` itself uses on its own Postgres/RustFS containers
(#385's "existing incident/outage scenario scaffolding, reused unmodified" acceptance
criterion), drives `load_driver.py` for the outage's duration, then polls the Shipper's
buffer size until it clears or a wait bound expires and records that elapsed time as the
drain time.

Buffer size comes from Vector's own `internal_metrics` + `prometheus_exporter`
(`vector_buffer_byte_size`), not a filesystem stat on the buffer directory — a discovery
made empirically while building this: Vector's on-disk buffer format (a segment-based
mmap log) does not shrink its `.dat` segment file as events are acked, so `du -sb` on the
buffer directory stays pinned at its post-outage peak indefinitely, which would make
every drain look like it never happened. The metrics gauge, by contrast, tracks the
buffer's actual pending bytes and reaches ~0 once the backlog genuinely clears. Injecting
a metrics source/sink into this standalone Vector config is not a DC code change — #323's
PRD anticipates exactly this ("observability comes from configuration, not code
changes").

Every outage-phase observation window is run through `saturation_probe.evaluate()`
(#377) with `outage_declared=True`, and the whole run hard-fails
(`OutageMisreportedError`) if that window is ever reported saturated — a growing buffer
during a declared outage must never be misreported, and this is the same module and the
same disk-buffer signal #377 defines, not a bespoke check. The *recovery*-phase window
reuses the identical signal with `outage_declared=False` to decide whether an outage
length is the ramp's knee — the shortest outage length whose backlog failed to drain
within the wait bound (never fabricated: a backlog that always drains within the tested
range is reported as `BOUND_NOT_FOUND`, matching #379's own rule). Output feeds
`curve_reporter.py` in the same stable `AxisRun`/`CurveReport` shape the other axes use;
`AxisRun.points[].level` is the outage length in seconds (the same unit
`tripped_level`/`highest_clear_level` come in) — the per-level *drain time* this axis
exists to report has no field of its own in that shared shape, so it travels alongside
the curve report as a companion JSON keyed by the same outage-length levels.
`tools/e2e/test/test_drain_rate_axis.py` covers the pure parts (`evaluate_cycle`,
`build_axis_run`, `find_drain_rate_curve`) with fake drivers and synthetic observation
windows, no containers; the real podman/`load_driver.py` orchestration is exercised by
running the scenario script itself, not unit-tested, matching the two-tier composer's own
testing decision.

Not wired into `ci.yaml`, same as the scenarios above.

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
- `scripts/saturation_probe.py` — the limits harness's saturation verdict (#377, part of
  #323): a pure function over a window of ack-latency/unacked-window-depth/disk-buffer
  observations, checked in the PRD's stated priority order and unit-tested alone, with
  no container of its own.
- `scripts/ramp_controller.py` — the limits harness's ramp controller (#379, part of
  #323): given a driver, a probe, and an ascending `RampPolicy`, drives load level by
  level until the probe trips and reports the knee, or `BOUND_NOT_FOUND` if it never
  does. No I/O, no container; unit-tested with fake drivers/probes alone.
- `scripts/curve_reporter.py` — the limits harness's curve reporter (#380, part of
  #323): a pure function turning a set of per-axis runs into a stable, diffable
  structured report plus a human-readable summary naming the binding constraint (CPU,
  memory, or disk) at saturation, with every point labelled measured or extrapolated.
  No I/O, no container; unit-tested against synthetic run data alone.
- `scripts/run_limits_two_tier.sh` / `params/e2e_limits_params.yaml` /
  `params/e2e_limits_forward_sink.toml` — the two-tier topology composer (#381, part of
  #323) described above: N real DC stacks + M synthetic senders through an aggregating
  Shipper to shared Postgres/RustFS, reusing `verify_zero_loss.py` unmodified.
- `scripts/single_robot_ceiling_driver.py` — the single-robot ceiling axis's driver
  adapter (#383, part of #323) described above: conforms to `ramp_controller.py`'s
  Driver interface and `load_driver.py`'s ledger-plus-latency return shape, sourced from
  polling the real stack. No I/O of its own (an injected `RealStackHandle`); unit-tested
  with a fake handle and a virtual clock alone.
- `scripts/run_limits_single_robot_ceiling.py` / `scripts/run_limits_single_robot_ceiling.sh`
  — the single-robot ceiling axis's orchestrator and topology script (#383, part of
  #323): the real-I/O `RealStackHandle`, the find_knee-result-to-`AxisRun` folding, and
  the one-real-stack podman topology, reusing `verify_zero_loss.py` unmodified after a
  post-ramp drain.
- `scripts/run_limits_shipper_fanin.sh` / `scripts/run_limits_shipper_fanin.py` — the
  Shipper fan-in axis (#382, part of #323) described above: ramping robot count against
  #381's two-tier topology to the knee and proving the instrument itself against a
  deliberately constrained aggregating Shipper.
- `scripts/upload_saturation_probe.py` — the Uploader concurrency axis's own saturation
  verdict (#384, part of #323): a pure function, sibling to `saturation_probe.py` but
  over queue-depth/verify-delete-backlog observations instead of ack latency, since
  Files never touch the Shipper. No I/O, no container; unit-tested alone.
- `scripts/run_upload_concurrency_axis.py` / `scripts/run_limits_upload_concurrency.sh` /
  `params/e2e_limits_upload_params.yaml` — the Uploader concurrency axis (#384, part of
  #323) described above: calls `ramp_controller.find_knee()` unchanged, ramping N
  Bridge/Uploader containers against shared Postgres/RustFS and asserting
  delete-after-verify ordering plus Group completion correctness at every level.
- `scripts/drain_rate_axis.py` / `scripts/run_limits_drain_rate.sh` — the drain-rate axis
  (#385, part of #323) described above: an induced-outage-then-recovery ramp over outage
  length at a fixed synthetic load, reusing run.sh's own outage-inducing recipe and
  saturation_probe.py's outage-aware disk-buffer signal, reporting into
  curve_reporter.py's stable shape. `evaluate_cycle()`/`build_axis_run()`/
  `find_drain_rate_curve()` are pure and unit-tested with fakes
  (`test_drain_rate_axis.py`); the podman/load_driver.py orchestration is exercised by
  running the scenario script.

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
