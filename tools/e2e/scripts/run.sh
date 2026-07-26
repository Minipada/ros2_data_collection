#!/bin/bash
# One-command zero-loss E2E harness (#249). From the repo root:
#
#   ./tools/e2e/scripts/run.sh
#
# Builds the images, brings up Postgres + RustFS + the full DC stack (podman compose),
# asserts launch-to-first-Record < 10s, runs a steady-state warmup, induces an outage
# (stops the destination containers — operationally equivalent to a network partition
# from dc_bridge/Vector's point of view: both surface as "destination unreachable",
# and both are recovered by the same disk-buffer-then-reconnect path, ADR-0002), does a
# full stack restart while destinations are still down, restores them, drains, then
# hard-asserts exactly-once/zero-loss (tools/e2e/scripts/verify_zero_loss.py). Every
# gate is a real, hard-failing assertion — nothing here is skip-on-missing (matching
# the #246 follow-up decision for dc_bridge's own store-backed tests: a verification
# that silently didn't run must never look identical to one that passed).
#
# Env vars (all optional):
#   DC_E2E_OUTAGE_SECONDS         outage duration (default 600 = the PRD's 10 minutes;
#                                 CI uses a shorter override — see ci-jazzy.yaml)
#   DC_E2E_STEADY_STATE_SECONDS   warmup before the outage (default 30)
#   DC_E2E_DRAIN_SECONDS          settle time after recovery, before verifying (default 30)
#   DC_E2E_STARTUP_TIMEOUT_SECONDS  startup-latency hard gate (default 10, per the PRD)
#   DC_E2E_KEEP                  "true" to leave the stack up after a failure for debugging
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run"

OUTAGE_SECONDS="${DC_E2E_OUTAGE_SECONDS:-600}"
STEADY_STATE_SECONDS="${DC_E2E_STEADY_STATE_SECONDS:-30}"
DRAIN_SECONDS="${DC_E2E_DRAIN_SECONDS:-30}"
STARTUP_TIMEOUT_SECONDS="${DC_E2E_STARTUP_TIMEOUT_SECONDS:-10}"
KEEP="${DC_E2E_KEEP:-false}"

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

log() { echo "[e2e $(date -u +%H:%M:%S)] $*"; }

STATS_PID=""
cleanup() {
  local exit_code=$?
  if [ -n "$STATS_PID" ]; then
    kill "$STATS_PID" 2>/dev/null || true
  fi
  if [ "$exit_code" -ne 0 ] && [ "$KEEP" = "true" ]; then
    log "FAILED (exit $exit_code) — leaving the stack up (DC_E2E_KEEP=true) for debugging"
    exit "$exit_code"
  fi
  log "tearing down"
  podman compose -f compose.yaml logs > "$RUN_DIR/compose.log" 2>&1 || true
  podman compose -f compose.yaml down -v --timeout 10 || true
  exit "$exit_code"
}
trap cleanup EXIT

pg_exec() {
  podman compose -f compose.yaml exec -T postgres psql -U dc -d dc -tAc "$1"
}

log "building the DC workspace image (tools/e2e/scripts/build.sh — the same build CI uses)"
"$SCRIPT_DIR/build.sh"

log "building the E2E harness's thin runtime layer (Containerfile.e2e, FROM the workspace image)"
podman build -t dc-e2e:latest -f Containerfile.e2e .

log "starting Postgres + RustFS"
podman compose -f compose.yaml up -d postgres rustfs
timeout 60 bash -c 'until podman compose -f compose.yaml exec -T postgres pg_isready -U dc >/dev/null 2>&1; do sleep 1; done' \
  || { log "Postgres never became ready"; exit 1; }
timeout 60 bash -c 'until curl -sf http://127.0.0.1:9000 >/dev/null 2>&1 || curl -s http://127.0.0.1:9000 >/dev/null 2>&1; do sleep 1; done' \
  || { log "RustFS never became ready"; exit 1; }

log "creating the RustFS bucket (dc_bridge's S3 sink doesn't create it — see dc_bridge/tests/end_to_end.rs)"
podman run --rm --network host docker.io/minio/mc:latest \
  sh -c 'mc alias set local http://127.0.0.1:9000 rustfsadmin rustfsadmin && mc mb -p local/dc-e2e'

# Resource usage (CPU/RSS) is informational per the PRD, not gating — sampled for the
# whole run so tools/e2e/scripts/measure_resources.sh's summary covers steady state.
"$SCRIPT_DIR/measure_resources.sh" "$RUN_DIR/resource_usage.csv" &
STATS_PID=$!

log "starting the DC stack — measuring launch-to-first-Record latency"
START_TS=$(date +%s.%N)
podman compose -f compose.yaml up -d dc

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

log "inducing outage: stopping Postgres + RustFS for ${OUTAGE_SECONDS}s (dc_bridge/Vector keep running and buffering to disk — ADR-0002)"
podman compose -f compose.yaml stop postgres rustfs
sleep "$OUTAGE_SECONDS"

log "full stack restart while destinations are still down (proves recovery doesn't depend on the Bridge having stayed up)"
podman compose -f compose.yaml restart dc

log "restoring Postgres + RustFS"
podman compose -f compose.yaml start postgres rustfs
timeout 60 bash -c 'until podman compose -f compose.yaml exec -T postgres pg_isready -U dc >/dev/null 2>&1; do sleep 1; done' \
  || { log "Postgres never came back"; exit 1; }

log "draining for ${DRAIN_SECONDS}s"
sleep "$DRAIN_SECONDS"

log "stopping the workload so counts settle before verification"
podman compose -f compose.yaml stop dc
sleep 5

kill "$STATS_PID" 2>/dev/null || true
STATS_PID=""

log "verifying zero-loss / exactly-once"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --compose-file "$E2E_DIR/compose.yaml" \
  --num-synth-topics 14 \
  --report "$RUN_DIR/verification_report.json"

log "PASS: zero-loss E2E harness"
if [ -f "$RUN_DIR/resource_usage.csv" ]; then
  log "resource usage summary (informational):"
  tail -5 "$RUN_DIR/resource_usage.csv"
fi
