#!/bin/bash
# One-command zero-loss E2E harness (#249). From the repo root:
#
#   ./tools/e2e/scripts/run.sh
#
# Brings up Postgres + RustFS + the full DC stack with plain `podman` (no compose — see
# CLAUDE.md "Containers: Podman, not Docker"; the outage/restart lifecycle below IS the
# test, and driving container lifecycle by hand is simpler and dependency-free than any
# compose provider). Asserts launch-to-first-Record < 10s, runs a steady-state warmup,
# induces an outage (stops the destination containers — operationally equivalent to a
# network partition from dc_bridge/Vector's point of view: both surface as "destination
# unreachable", and both are recovered by the same disk-buffer-then-reconnect path,
# ADR-0002), does a full stack restart while destinations are still down, restores them,
# drains, then hard-asserts zero-loss (tools/e2e/scripts/verify_zero_loss.py). The shipper
# is at-least-once (ADR-0002), so a record in-flight at outage time can be re-sent on
# recovery — the verifier deduplicates on the natural key (exactly-once on read) and
# reports such boundary re-sends as notes; only actual loss fails the run. The
# startup-latency and zero-loss gates are real, hard-failing assertions — nothing here is
# skip-on-missing (matching the #246 follow-up decision for dc_bridge's own store-backed
# tests: a verification that silently didn't run must never look identical to one that
# passed).
#
# Env vars (all optional):
#   DC_E2E_OUTAGE_SECONDS         outage duration (default 600 = the PRD's 10 minutes;
#                                 CI uses a shorter override — see ci.yaml)
#   DC_E2E_STEADY_STATE_SECONDS   warmup before the outage (default 30)
#   DC_E2E_DRAIN_SECONDS          settle time after recovery, before verifying (default 30)
#   DC_E2E_STARTUP_TIMEOUT_SECONDS  startup-latency hard gate (default 10, per the PRD)
#   DC_E2E_KEEP                  "true" to leave the stack up after a failure for debugging
#   DC_E2E_IMAGE                 a prebuilt dc-e2e image ref to run as the DC stack instead
#                                of building it here. CI's e2e job sets this to the :<sha>
#                                image its build-e2e-image job built and pushed (which is
#                                itself FROM the tested dc-workspace image) — so the harness
#                                runs the exact built artifact, no build at run time (one
#                                job builds, another uses). Unset locally: the image is
#                                built from the working tree (see DC_WORKSPACE_IMAGE).
#   DC_WORKSPACE_IMAGE           a prebuilt DC workspace image to use as the base for the
#                                locally-built dc-e2e image (skips build.sh). Ignored when
#                                DC_E2E_IMAGE is set. Unset: build.sh builds it.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run"

OUTAGE_SECONDS="${DC_E2E_OUTAGE_SECONDS:-600}"
STEADY_STATE_SECONDS="${DC_E2E_STEADY_STATE_SECONDS:-30}"
DRAIN_SECONDS="${DC_E2E_DRAIN_SECONDS:-30}"
STARTUP_TIMEOUT_SECONDS="${DC_E2E_STARTUP_TIMEOUT_SECONDS:-10}"
KEEP="${DC_E2E_KEEP:-false}"

# Container + volume names. Volumes are named (not host bind-mounts) so a full stack
# restart (podman restart / stop+start) preserves dc_bridge's on-disk buffer
# (shipper.data_dir) and Postgres's data directory — that persistence is what makes the
# zero-loss guarantee (ADR-0002) survive a restart, not just a live process.
PG_C=dc_e2e_postgres
RUSTFS_C=dc_e2e_rustfs
DC_C=dc_e2e_dc
VOLUMES=(dc_e2e_pgdata dc_e2e_rustfs_data dc_e2e_buffer dc_e2e_data)

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

log() { echo "[e2e $(date -u +%H:%M:%S)] $*"; }

remove_stack() {
  # --ignore makes "no such container" a clean success while still surfacing real errors
  # (a broken podman, a container that won't die); likewise only remove volumes that exist.
  podman rm -f --ignore "$DC_C" "$PG_C" "$RUSTFS_C" >/dev/null
  for v in "${VOLUMES[@]}"; do
    if podman volume exists "$v"; then
      podman volume rm "$v" >/dev/null
    fi
  done
}

pg_exec() {
  podman exec "$PG_C" psql -U dc -d dc -tAc "$1"
}

STATS_PID=""
cleanup() {
  local exit_code=$?
  if [ -n "$STATS_PID" ] && kill -0 "$STATS_PID" 2>/dev/null; then
    kill "$STATS_PID"
  fi
  if [ "$exit_code" -ne 0 ] && [ "$KEEP" = "true" ]; then
    log "FAILED (exit $exit_code) — leaving the stack up (DC_E2E_KEEP=true) for debugging"
    exit "$exit_code"
  fi
  log "tearing down"
  if podman container exists "$DC_C"; then
    podman logs "$DC_C" > "$RUN_DIR/dc.log" 2>&1
  fi
  remove_stack
  exit "$exit_code"
}
trap cleanup EXIT

# --- obtain the DC stack image ------------------------------------------------------
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

# Clean any leftovers from a previous (possibly DC_E2E_KEEP=true) run.
remove_stack

# --- bring up the destinations ------------------------------------------------------
log "starting Postgres + RustFS"
podman run -d --network host --name "$PG_C" \
  -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
  -v dc_e2e_pgdata:/var/lib/postgresql/data \
  -v "$E2E_DIR/sql/init.sql:/docker-entrypoint-initdb.d/init.sql:ro" \
  docker.io/library/postgres:13 >/dev/null
# rustfs/rustfs 1.0.0-beta.11 — pinned by digest, not :latest, so an upstream image
# change can't silently alter harness behavior. Bump deliberately: check
# https://hub.docker.com/r/rustfs/rustfs/tags for the new digest.
podman run -d --network host --name "$RUSTFS_C" \
  -v dc_e2e_rustfs_data:/data \
  docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c >/dev/null

timeout 60 bash -c "until podman exec $PG_C pg_isready -U dc >/dev/null 2>&1; do sleep 1; done" \
  || { log "Postgres never became ready"; exit 1; }
timeout 60 bash -c 'until curl -sf http://127.0.0.1:9000 >/dev/null 2>&1 || curl -s http://127.0.0.1:9000 >/dev/null 2>&1; do sleep 1; done' \
  || { log "RustFS never became ready"; exit 1; }

log "creating the RustFS bucket (the S3 sink / Uploader doesn't create it)"
# The AWS CLI talks plain S3 to RustFS via --endpoint-url — same protocol dc_bridge's
# Uploader uses (aws-sdk-cpp), so no extra vendor (the retired minio/mc client) in the loop.
podman run --rm --network host \
  -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
  docker.io/amazon/aws-cli:latest \
  --endpoint-url http://127.0.0.1:9000 s3 mb s3://dc-e2e

# Resource usage (CPU/RSS) is informational per the PRD, not gating — sampled for the
# whole run so tools/e2e/scripts/measure_resources.sh's summary covers steady state.
"$SCRIPT_DIR/measure_resources.sh" "$RUN_DIR/resource_usage.csv" &
STATS_PID=$!

# --- start the DC stack, measure launch-to-first-Record -----------------------------
log "starting the DC stack — measuring launch-to-first-Record latency"
START_TS=$(date +%s.%N)
podman run -d --network host --name "$DC_C" \
  -v dc_e2e_buffer:/root/.dc/e2e/buffer \
  -v dc_e2e_data:/root/.dc/e2e/data \
  "$DC_IMAGE" >/dev/null

FIRST_RECORD_LATENCY=""
DEADLINE=$(echo "$START_TS + $STARTUP_TIMEOUT_SECONDS + 5" | bc)
while (( $(echo "$(date +%s.%N) < $DEADLINE" | bc) )); do
  COUNT="$(pg_exec 'SELECT count(*) FROM dc_records' 2>/dev/null || echo 0)"
  if [ "${COUNT:-0}" -gt 0 ] 2>/dev/null; then
    FIRST_RECORD_LATENCY=$(echo "$(date +%s.%N) - $START_TS" | bc)
    break
  fi
  sleep 0.2
done

if [ -z "$FIRST_RECORD_LATENCY" ]; then
  log "FAIL: no Record landed in Postgres within $((STARTUP_TIMEOUT_SECONDS + 5))s of starting the stack"
  exit 1
fi
log "first Record landed after ${FIRST_RECORD_LATENCY}s"
if (( $(echo "$FIRST_RECORD_LATENCY > $STARTUP_TIMEOUT_SECONDS" | bc) )); then
  log "FAIL: startup latency ${FIRST_RECORD_LATENCY}s exceeds the ${STARTUP_TIMEOUT_SECONDS}s gate"
  exit 1
fi
log "PASS: startup latency gate (<${STARTUP_TIMEOUT_SECONDS}s)"

log "steady state for ${STEADY_STATE_SECONDS}s"
sleep "$STEADY_STATE_SECONDS"

# --- outage → restart → restore -----------------------------------------------------
log "inducing outage: stopping Postgres + RustFS for ${OUTAGE_SECONDS}s (dc_bridge/Vector keep running and buffering to disk — ADR-0002)"
podman stop "$PG_C" "$RUSTFS_C" >/dev/null
sleep "$OUTAGE_SECONDS"

log "full stack restart while destinations are still down (proves recovery doesn't depend on the Bridge having stayed up)"
podman restart "$DC_C" >/dev/null

log "restoring Postgres + RustFS"
podman start "$PG_C" "$RUSTFS_C" >/dev/null
timeout 60 bash -c "until podman exec $PG_C pg_isready -U dc >/dev/null 2>&1; do sleep 1; done" \
  || { log "Postgres never came back"; exit 1; }

log "draining for ${DRAIN_SECONDS}s"
sleep "$DRAIN_SECONDS"

log "stopping the workload so counts settle before verification"
podman stop "$DC_C" >/dev/null
sleep 5

if [ -n "$STATS_PID" ] && kill -0 "$STATS_PID" 2>/dev/null; then
  kill "$STATS_PID"
fi
STATS_PID=""

# --- durable upload intent queue (#265) ----------------------------------------------
# The outage+restart above is this harness's stand-in for #265's own e2e acceptance
# criterion (publish a files-Destination Record with the store down, restart the whole
# Bridge process, bring the store back up, confirm the upload completes) — the camera
# Measurement's File already goes through that exact path every run, and
# verify_zero_loss.py's check_files() already asserts its file_status/group_complete
# rows landed with no duplicate "uploaded" row. What that alone can't prove is that the
# *queue itself* drained rather than leaving an orphaned intent behind (the pre-#265
# in-memory queue would have silently forgotten anything pending across the
# `podman restart` above) — so assert the on-disk queue is empty via the same named
# volume dc_bridge writes it to, overriding the image's entrypoint for a one-off `find`.
log "verifying the durable upload intent queue drained (no orphaned intents after the outage+restart)"
LEFTOVER_INTENTS=$(podman run --rm --entrypoint bash \
  -v dc_e2e_buffer:/vol:ro "$DC_IMAGE" \
  -c 'find /vol/queue/upload -maxdepth 1 -name "*.json" 2>/dev/null | wc -l')
if [ "${LEFTOVER_INTENTS:-0}" -ne 0 ]; then
  log "FAIL: ${LEFTOVER_INTENTS} orphaned upload intent(s) left in the queue after the run"
  exit 1
fi
log "PASS: upload intent queue empty (0 orphaned intents)"

# --- verify -------------------------------------------------------------------------
log "verifying zero-loss (duplicates from the at-least-once boundary are deduped on read)"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --postgres-container "$PG_C" \
  --num-synth-topics 14 \
  --report "$RUN_DIR/verification_report.json"

log "PASS: zero-loss E2E harness"
if [ -f "$RUN_DIR/resource_usage.csv" ]; then
  log "resource usage summary (informational):"
  tail -5 "$RUN_DIR/resource_usage.csv"
fi
