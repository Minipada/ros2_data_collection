#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Limits harness: two-tier topology composer (#381, part of #323's epic). From the repo
# root:
#
#   ./tools/e2e/scripts/run_limits_two_tier.sh
#
# Brings up, with plain podman on a dedicated bridge network (container-name DNS, same
# convention as run_degraded.sh — see that script's header for why not --network host:
# N real DC stacks all binding the same default Bridge-to-Shipper port would collide on a
# shared host network namespace): a shared Postgres + RustFS (the real Destinations), an
# aggregating Shipper (a standalone Vector instance, no dc_bridge/ROS involved), N real DC
# stacks (each its own dc_bringup + dc_bridge + local Shipper,
# params/e2e_limits_params.yaml), and M synthetic senders (scripts/load_driver.py, #378).
# At a fixed, non-saturating load, it then reuses verify_zero_loss.py UNMODIFIED to prove
# Records survive the extra hop.
#
# --- The two-tier chain ------------------------------------------------------------------
#
# Each real stack's own local Shipper (its Bridge-spawned Vector, unconditional per
# dc_bridge/src/bridge_node.cpp — every Bridge always runs one) has no Destination that
# writes Records to the real Postgres at all: params/e2e_limits_params.yaml replaces the
# usual `pgsql_records` with `local_sink`, a throwaway `file` Destination whose only job
# is to make dc_bridge render the `dc.<tag>` route branches those topics need.
# params/e2e_limits_forward_sink.toml (ADR-0003 `custom_config_files`, the same
# passthrough mechanism e2e_passthrough_sink.toml/e2e_mcap_sink.toml already use) wires a
# `type = "vector"` sink to those same route branches — Vector's own native relay
# protocol, the standard way to chain a local ("agent") Vector to a central
# ("aggregator") one — forwarding them over the network to this script's aggregating
# Shipper instead. That relay was verified directly against the pinned Vector build
# before this script was written: a `fluent` source -> `vector` sink -> `vector` source
# -> `file` sink chain relayed scripts/load_driver.py frames end to end with every custom
# field intact, including the ack round trip back to the sender. The aggregating
# Shipper's own `postgres` sink is the only thing that ever writes Measurement/synth
# Records into the real `dc_records` table — no real stack's own local Shipper has a
# direct path there, standing in for #323's motivating scenario: "a private site where
# robots have no internet, and an edge server aggregates ... and forwards upstream."
#
# The M synthetic senders (one scripts/load_driver.py invocation, `--connections M`) hit
# the aggregating Shipper's own `fluent` source directly, alongside the `vector` source
# tier-1 relays into — the same aggregating Shipper the real stacks use, adding the
# realistic connection count and byte rate #323's PRD calls for. Their frames aren't
# dc_records-shaped (see e2e_limits_forward_sink.toml's header), so they land on a
# separate `blackhole` sink rather than the real Postgres path; the driver's own
# sent/acked ledger (checked below) is their zero-loss proof instead of a Postgres row
# check.
#
# --- Discovery recorded per #381's acceptance criteria -----------------------------------
#
# This composer only routes `receives: records` Measurement Records through the two-tier
# chain. The Uploader — File bytes over its own AWS SDK client, plus its own
# file-metadata Records — talks directly to whatever `s3`/`postgres` Destination host is
# configured, with no Vector hop at all (dc_bridge/src/uploader_main.cpp's dc_uploader
# process never touches the rendered Vector config). There is no way to route
# Files through the aggregating Shipper without new DC code, so
# params/e2e_limits_params.yaml points the Files-side Destinations straight at the real
# shared Postgres/RustFS instead of through the chain. Extending true two-tier coverage
# to Files is out of scope here, left as a follow-up.
#
# Second discovery: the two-tier design concentrates every real stack's Records through
# one aggregating Shipper's one `postgres` sink — a shared bottleneck no single-tier
# scenario (run.sh et al., each stack writing straight to its own Postgres connection)
# has. Verified empirically here: two real stacks (REAL_STACKS=2) reliably produced far
# more at-least-once re-delivery than one does — past what verify_zero_loss.py's
# zero-tolerance memory-timestamp-uniqueness check (check_timestamp_resolution) accepts.
# Vector's own end-to-end acknowledgement retries a chunk whose ack didn't land inside its
# window, and the busier shared sink under N>1 was slow enough here to trip that. Not a
# correctness bug — every value still arrived; the "duplicates" are identical (date,
# value) pairs, genuine at-least-once repeats, not loss (REAL_STACKS=1's own clean run and
# a direct Postgres query both confirm it) — and not something to silently work around:
# REAL_STACKS stays configurable rather than hardcoded, since finding exactly where that
# bottleneck sits is squarely the ramp controller's job (#323's next piece), not this
# composer's. Default kept at 1 real stack so this script passes reliably on its own, per
# #381's own "before any ramping logic exists, at a fixed, non-saturating load" scope.
#
# Env vars (all optional):
#   DC_E2E_LIMITS_REAL_STACKS           number of real DC stacks (default 1 — see the
#                                        second discovery above before raising this)
#   DC_E2E_LIMITS_SYNTH_CONNECTIONS     load_driver.py connection count (default 50)
#   DC_E2E_LIMITS_SYNTH_RATE_HZ         load_driver.py per-connection Record rate (default 5)
#   DC_E2E_LIMITS_STEADY_STATE_SECONDS  fixed-load window duration (default 30)
#   DC_E2E_LIMITS_DRAIN_SECONDS   settle time after the window, before verifying (default 15)
#   DC_E2E_KEEP                  "true" to leave the stack (and its network) up after a
#                                 failure for debugging
#   DC_E2E_IMAGE / DC_WORKSPACE_IMAGE   same meaning as run.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(cd "$E2E_DIR/../.." && pwd)"
RUN_DIR="$E2E_DIR/.run/limits_two_tier"

REAL_STACKS="${DC_E2E_LIMITS_REAL_STACKS:-1}"
SYNTH_CONNECTIONS="${DC_E2E_LIMITS_SYNTH_CONNECTIONS:-50}"
SYNTH_RATE_HZ="${DC_E2E_LIMITS_SYNTH_RATE_HZ:-5}"
STEADY_STATE_SECONDS="${DC_E2E_LIMITS_STEADY_STATE_SECONDS:-30}"
# The camera Measurement fires every 15s (params/e2e_limits_params.yaml) and its capture
# still has to clear the Uploader's own capture/queue/upload/verify pipeline after the
# steady-state window ends — verified empirically to matter: without this drain, a run
# landing its last capture near the end of the window intermittently stopped the stack
# before that capture's dc_files metadata row landed, failing check_files() on an
# otherwise-correct run.
DRAIN_SECONDS="${DC_E2E_LIMITS_DRAIN_SECONDS:-15}"
KEEP="${DC_E2E_KEEP:-false}"

NET=dc-e2e-limits-net
# Hyphens, not the underscored dc_e2e_* convention every other scenario script uses:
# these names are embedded in URLs (RustFS/Postgres endpoints, the vector-relay address)
# and aws-cli 2.36's --endpoint-url parser rejects a hostname with an underscore outright
# (verified empirically — the same underscored convention on run_degraded.sh's own
# RustFS container name would hit this identically). Volume names below are unaffected
# (never used as a hostname) and keep the usual underscored convention.
PG_C=dc-e2e-limits-postgres
RUSTFS_C=dc-e2e-limits-rustfs
AGG_C=dc-e2e-limits-agg
AGG_VECTOR_PORT=6000
AGG_FLUENT_PORT=24224
STACK_PREFIX=dc-e2e-limits-stack-
# Stack 0 is the ledger-checked stack: verify_zero_loss.py's contract is one ledger per
# run. The other real stacks and the synthetic senders are unverified-by-ledger
# background load — exactly the PRD's "real stacks plus synthetic senders together"
# nominal-load mix, all going through the same aggregating Shipper as stack 0.
LEDGER_STACK_INDEX=0

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

log() { echo "[e2e-limits $(date -u +%H:%M:%S)] $*"; }

stack_name() { echo "${STACK_PREFIX}$1"; }

all_stack_names() {
  local i
  for ((i = 0; i < REAL_STACKS; i++)); do
    stack_name "$i"
  done
}

remove_stack() {
  # shellcheck disable=SC2046
  podman rm -f --ignore $(all_stack_names) "$AGG_C" "$PG_C" "$RUSTFS_C" >/dev/null
  local i v
  for ((i = 0; i < REAL_STACKS; i++)); do
    for v in "dc_e2e_limits_buffer_$i" "dc_e2e_limits_data_$i"; do
      if podman volume exists "$v"; then
        podman volume rm "$v" >/dev/null
      fi
    done
  done
  for v in dc_e2e_limits_pgdata dc_e2e_limits_rustfs_data; do
    if podman volume exists "$v"; then
      podman volume rm "$v" >/dev/null
    fi
  done
  podman network rm "$NET" >/dev/null 2>&1 || true
}

cleanup() {
  local exit_code=$?
  if [ "$exit_code" -ne 0 ] && [ "$KEEP" = "true" ]; then
    log "FAILED (exit $exit_code) — leaving the stack up (DC_E2E_KEEP=true) for debugging"
    exit "$exit_code"
  fi
  log "tearing down"
  local s
  for s in $(all_stack_names); do
    if podman container exists "$s"; then
      podman logs "$s" > "$RUN_DIR/${s}.log" 2>&1
    fi
  done
  if podman container exists "$AGG_C"; then
    podman logs "$AGG_C" > "$RUN_DIR/agg.log" 2>&1
  fi
  remove_stack
  exit "$exit_code"
}
trap cleanup EXIT

# --- obtain the DC stack image (shared with run.sh — see its header) ------------------
if [ -n "${DC_E2E_IMAGE:-}" ]; then
  log "using prebuilt E2E image: $DC_E2E_IMAGE (no build)"
  podman image exists "$DC_E2E_IMAGE" || podman pull "$DC_E2E_IMAGE"
  DC_IMAGE="$DC_E2E_IMAGE"
else
  if [ -n "${DC_WORKSPACE_IMAGE:-}" ]; then
    log "using prebuilt DC workspace image: $DC_WORKSPACE_IMAGE (skipping build.sh)"
    podman image exists "$DC_WORKSPACE_IMAGE" || podman pull "$DC_WORKSPACE_IMAGE"
    WORKSPACE_IMAGE="$DC_WORKSPACE_IMAGE"
  else
    log "building the DC workspace image (tools/e2e/scripts/build.sh — the same build CI uses)"
    "$SCRIPT_DIR/build.sh"
    WORKSPACE_IMAGE="dc-workspace:latest"
  fi
  log "building the E2E image (Containerfile.e2e, FROM the workspace image)"
  podman build --build-arg "BASE_IMAGE=$WORKSPACE_IMAGE" -t dc-e2e:latest -f Containerfile.e2e .
  DC_IMAGE="dc-e2e:latest"
fi

# The exact Vector version dc_bridge is built and tested against, for the standalone
# aggregating Shipper — read out of the .repos pin that fetches vector_vendor (same as
# run_load_driver_shipper_test.sh), not duplicated as a second source of truth.
VECTOR_VERSION="$(grep -A3 'vector_vendor:' "$REPO_ROOT/ros2_data_collection.repos" | grep -oP 'version: v\K[0-9.]+')"
VECTOR_IMAGE="docker.io/timberio/vector:${VECTOR_VERSION}-debian"

# Clean any leftovers from a previous (possibly DC_E2E_KEEP=true) run.
remove_stack

# --- bridge network + shared destinations ----------------------------------------------
log "creating the bridge network ($NET)"
podman network create "$NET" >/dev/null

log "starting the shared Postgres + RustFS on $NET"
podman run -d --network "$NET" --name "$PG_C" \
  -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
  -v dc_e2e_limits_pgdata:/var/lib/postgresql/data \
  -v "$E2E_DIR/sql/init.sql:/docker-entrypoint-initdb.d/init.sql:ro" \
  docker.io/library/postgres:13 >/dev/null
podman run -d --network "$NET" --name "$RUSTFS_C" \
  -v dc_e2e_limits_rustfs_data:/data \
  docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c >/dev/null

timeout 120 bash -c "until podman exec $PG_C psql -U dc -d dc -tAc \"SELECT to_regclass('public.dc_records')\" 2>/dev/null | grep -q dc_records; do sleep 2; done" \
  || { log "FAIL: Postgres never came up with sql/init.sql applied"; exit 1; }

log "waiting for RustFS to accept TCP connections on $NET"
# --entrypoint bash: the image's own ENTRYPOINT (entrypoint.sh) launches the full DC
# stack and appends any extra args to `ros2 launch` rather than running them as a
# separate command, so a one-off diagnostic run needs an explicit override (verified
# empirically) — a plain bash /dev/tcp probe rather than scripts/measure_rtt.py, so this
# doesn't depend on that file happening to be baked into whatever $DC_IMAGE is passed in.
timeout 60 bash -c "
  until podman run --rm --network $NET --entrypoint bash '$DC_IMAGE' -c 'exec 3<>/dev/tcp/$RUSTFS_C/9000' >/dev/null 2>&1; do
    sleep 1
  done
" || { log "FAIL: RustFS never became reachable on $NET"; exit 1; }

log "creating the RustFS bucket"
podman run --rm --network "$NET" \
  -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
  docker.io/amazon/aws-cli:latest \
  --endpoint-url "http://$RUSTFS_C:9000" s3 mb s3://dc-e2e

# --- the aggregating Shipper: a standalone Vector instance, no dc_bridge involved -------
log "rendering the aggregating Shipper's Vector config"
cat > "$RUN_DIR/aggregator_vector.toml" <<EOF
data_dir = "/var/lib/vector"

[acknowledgements]
enabled = true

# Tier 1: real DC stacks' own local Shippers relay their dc.<tag> route branches here
# over Vector's native inter-instance protocol (see e2e_limits_forward_sink.toml).
[sources.from_tier1]
type = "vector"
address = "0.0.0.0:${AGG_VECTOR_PORT}"

# The M synthetic senders (#378's load_driver.py) speak the same shipper ingest protocol
# a real Bridge's Forwarder does — this is the same fluent listener shape
# run_load_driver_shipper_test.sh already proves load_driver.py is byte-compatible with.
[sources.from_synth]
type = "fluent"
address = "0.0.0.0:${AGG_FLUENT_PORT}"

# The only thing that writes Measurement/synth Records into the real dc_records table —
# see this script's header for why no real stack's own local Shipper does.
[sinks.to_postgres]
type = "postgres"
inputs = ["from_tier1"]
endpoint = "postgres://dc:password@${PG_C}:5432/dc"
table = "dc_records"

[sinks.to_postgres.buffer]
type = "disk"
max_size = 268435488

# Synthetic frames aren't dc_records-shaped (connection_id/seq, not date/tag/...) — they
# exist to add realistic connection count and byte rate (#323's PRD), not to be
# individually verified in Postgres. Kept off the real Records path so a schema mismatch
# there can never touch it; the driver's own sent/acked ledger is their zero-loss proof.
[sinks.discard_synth]
type = "blackhole"
inputs = ["from_synth"]
EOF

log "starting the aggregating Shipper ($AGG_C)"
podman run -d --network "$NET" -p "${AGG_FLUENT_PORT}:${AGG_FLUENT_PORT}" --name "$AGG_C" \
  -v "$RUN_DIR/aggregator_vector.toml:/etc/vector/vector.toml:Z" \
  "$VECTOR_IMAGE" -c /etc/vector/vector.toml >/dev/null

log "waiting for the aggregating Shipper's synthetic-sender ingress (fluent, published on the host) to accept connections"
if ! timeout 30 bash -c "until (exec 3<>/dev/tcp/127.0.0.1/${AGG_FLUENT_PORT}) 2>/dev/null; do sleep 0.5; done"; then
  log "FAIL: the aggregating Shipper's fluent source never started listening on port $AGG_FLUENT_PORT"
  exit 1
fi

log "waiting for the aggregating Shipper's tier-1 ingress (vector protocol, $NET-only) to accept connections"
timeout 60 bash -c "
  until podman run --rm --network $NET --entrypoint bash '$DC_IMAGE' -c 'exec 3<>/dev/tcp/$AGG_C/$AGG_VECTOR_PORT' >/dev/null 2>&1; do
    sleep 1
  done
" || { log "FAIL: the aggregating Shipper's vector-protocol source never became reachable on $NET"; exit 1; }

# --- N real DC stacks --------------------------------------------------------------------
log "starting $REAL_STACKS real DC stack(s) against params/e2e_limits_params.yaml"
for ((i = 0; i < REAL_STACKS; i++)); do
  s="$(stack_name "$i")"
  podman volume create "dc_e2e_limits_buffer_$i" >/dev/null
  podman volume create "dc_e2e_limits_data_$i" >/dev/null
  podman run -d --network "$NET" --name "$s" \
    -v "dc_e2e_limits_buffer_$i:/root/.dc/e2e/buffer" \
    -v "dc_e2e_limits_data_$i:/root/.dc/e2e/data" \
    -v "$E2E_DIR/params/e2e_limits_params.yaml:/opt/e2e/e2e_params.yaml:ro" \
    -v "$E2E_DIR/params/e2e_limits_forward_sink.toml:/opt/e2e/e2e_limits_forward_sink.toml:ro" \
    "$DC_IMAGE" >/dev/null
  # Staggered, not all at once: reduces simultaneous ROS/DDS-discovery and lifecycle
  # bring-up contention across N concurrently-starting stacks. Does not by itself fix
  # #381's other recorded discovery below (REAL_STACKS default) — that one is a runtime
  # contention effect, not a startup-ordering one.
  sleep 5
done

log "waiting for the first Record to reach Postgres through the two-tier chain"
timeout 90 bash -c "until [ \"\$(podman exec $PG_C psql -U dc -d dc -tAc 'SELECT count(*) FROM dc_records' 2>/dev/null || echo 0)\" -gt 0 ] 2>/dev/null; do sleep 1; done" \
  || { log "FAIL: no Record reached Postgres through the two-tier chain within 90s"; exit 1; }
log "PASS: at least one Record reached Postgres through the aggregating Shipper"

# --- fixed nominal-load window: real stacks (already running) + M synthetic senders -----
log "running the fixed nominal-load window for ${STEADY_STATE_SECONDS}s: $REAL_STACKS real stack(s) + $SYNTH_CONNECTIONS synthetic connection(s) @ ${SYNTH_RATE_HZ}Hz"
SUMMARY_JSON="$(cd "$REPO_ROOT" && uv run --frozen python3 tools/e2e/scripts/load_driver.py \
  127.0.0.1 "$AGG_FLUENT_PORT" \
  --connections "$SYNTH_CONNECTIONS" \
  --rate "$SYNTH_RATE_HZ" \
  --duration "$STEADY_STATE_SECONDS" \
  --tag "dc.e2e.limits_synth")"
echo "$SUMMARY_JSON" > "$RUN_DIR/synth_summary.json"
log "synthetic sender summary: $SUMMARY_JSON"

UNACKED="$(python3 -c "import json,sys; print(json.load(sys.stdin)['unacked'])" <<<"$SUMMARY_JSON")"
if [ "$UNACKED" -ne 0 ]; then
  log "FAIL: $UNACKED of the synthetic sender's frame(s) were never acknowledged by the aggregating Shipper under nominal load"
  exit 1
fi
log "PASS: every synthetic frame was accepted and acknowledged by the aggregating Shipper"

log "draining for ${DRAIN_SECONDS}s (lets the last camera capture clear the Uploader pipeline)"
sleep "$DRAIN_SECONDS"

log "stopping the real DC stacks so counts settle before verification"
for s in $(all_stack_names); do
  podman stop "$s" >/dev/null
done
sleep 5

# --- extract the ledger-checked stack's output, same pattern as run.sh ------------------
DATA_VOL="dc_e2e_limits_data_${LEDGER_STACK_INDEX}"

log "extracting stack $LEDGER_STACK_INDEX's passthrough sink output"
podman run --rm --entrypoint bash \
  -v "$DATA_VOL:/vol:ro" "$DC_IMAGE" \
  -c 'cat /vol/passthrough/records.ndjson 2>/dev/null || true' > "$RUN_DIR/passthrough.ndjson"

log "extracting stack $LEDGER_STACK_INDEX's raw mode output"
podman run --rm --entrypoint bash \
  -v "$DATA_VOL:/vol:ro" "$DC_IMAGE" \
  -c 'cat /vol/raw/records.ndjson 2>/dev/null || true' > "$RUN_DIR/raw.ndjson"

log "summarizing stack $LEDGER_STACK_INDEX's MCAP passthrough writer output"
podman run --rm --entrypoint python3 \
  -v "$DATA_VOL:/vol:ro" "$DC_IMAGE" \
  /opt/e2e/mcap_summary.py /vol/mcap > "$RUN_DIR/mcap_summary.json"

log "extracting stack $LEDGER_STACK_INDEX's workload ledger"
podman run --rm --entrypoint bash \
  -v "$DATA_VOL:/vol:ro" "$DC_IMAGE" \
  -c 'cat /vol/workload_ledger.txt 2>/dev/null || true' > "$RUN_DIR/workload_ledger.txt"

# --- verify (verify_zero_loss.py's own logic, reused unmodified) ------------------------
log "verifying zero-loss against stack $LEDGER_STACK_INDEX's ledger, through the two-tier chain"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --postgres-container "$PG_C" \
  --num-synth-topics 14 \
  --ledger-file "$RUN_DIR/workload_ledger.txt" \
  --passthrough-file "$RUN_DIR/passthrough.ndjson" \
  --mcap-summary-file "$RUN_DIR/mcap_summary.json" \
  --raw-file "$RUN_DIR/raw.ndjson" \
  --report "$RUN_DIR/verification_report.json"

log "PASS: zero-loss E2E harness through the two-tier topology (#381)"
