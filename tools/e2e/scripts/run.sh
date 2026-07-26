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
# hard-asserts zero-loss (tools/e2e/scripts/verify_zero_loss.py). The shipper is
# at-least-once (ADR-0002), so a record in-flight at outage time can be re-sent on
# recovery — the verifier deduplicates on the natural key (exactly-once on read) and
# reports such boundary re-sends as notes; only actual loss fails the run. The
# startup-latency and zero-loss gates are real, hard-failing assertions — nothing here is
# skip-on-missing (matching the #246 follow-up decision for dc_bridge's own store-backed
# tests: a verification that silently didn't run must never look identical to one that
# passed).
#
# Env vars (all optional):
#   DC_E2E_OUTAGE_SECONDS         outage duration (default 600 = the PRD's 10 minutes;
#                                 CI uses a shorter override — see ci-jazzy.yaml)
#   DC_E2E_STEADY_STATE_SECONDS   warmup before the outage (default 30)
#   DC_E2E_DRAIN_SECONDS          settle time after recovery, before verifying (default 30)
#   DC_E2E_STARTUP_TIMEOUT_SECONDS  startup-latency hard gate (default 10, per the PRD)
#   DC_E2E_KEEP                  "true" to leave the stack up after a failure for debugging
#   DC_WORKSPACE_IMAGE           a prebuilt DC workspace image ref to use as the harness
#                                base instead of building one locally. CI's e2e job sets
#                                this to the :<sha> image its build-and-test job already
#                                built, tested, and pushed — so the harness runs against
#                                the exact tested artifact rather than rebuilding it (one
#                                job builds, the other uses). Unset locally: build.sh
#                                builds dc-workspace:latest from the working tree.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run"

# Pin the compose provider to podman-compose. `podman compose` otherwise prefers the
# docker-compose cli-plugin when present, which talks to a Docker-compatible API socket
# (the rootless Podman socket) that isn't running in a bare shell — it fails with
# "daemon not running". podman-compose shells out to `podman` directly: no socket, no
# Docker binary in the loop (CLAUDE.md "Containers: Podman, not Docker"). Same provider
# locally and in CI.
export PODMAN_COMPOSE_PROVIDER=podman-compose
if ! command -v podman-compose >/dev/null 2>&1; then
  echo "podman-compose is required (pip install podman-compose / apt install podman-compose)" >&2
  exit 1
fi

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

if [ -n "${DC_WORKSPACE_IMAGE:-}" ]; then
  log "using prebuilt DC workspace image: $DC_WORKSPACE_IMAGE (skipping build.sh)"
  podman image exists "$DC_WORKSPACE_IMAGE" || podman pull "$DC_WORKSPACE_IMAGE"
  WORKSPACE_IMAGE="$DC_WORKSPACE_IMAGE"
else
  log "building the DC workspace image (tools/e2e/scripts/build.sh — the same build CI uses)"
  "$SCRIPT_DIR/build.sh"
  WORKSPACE_IMAGE="dc-workspace:latest"
fi

log "building the E2E harness's thin runtime layer (Containerfile.e2e, FROM the workspace image)"
podman build --build-arg "BASE_IMAGE=$WORKSPACE_IMAGE" -t dc-e2e:latest -f Containerfile.e2e .

log "starting Postgres + RustFS"
podman compose -f compose.yaml up -d postgres rustfs
timeout 60 bash -c 'until podman compose -f compose.yaml exec -T postgres pg_isready -U dc >/dev/null 2>&1; do sleep 1; done' \
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

log "verifying zero-loss (duplicates from the at-least-once boundary are deduped on read)"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --compose-file "$E2E_DIR/compose.yaml" \
  --num-synth-topics 14 \
  --report "$RUN_DIR/verification_report.json"

log "PASS: zero-loss E2E harness"
if [ -f "$RUN_DIR/resource_usage.csv" ]; then
  log "resource usage summary (informational):"
  tail -5 "$RUN_DIR/resource_usage.csv"
fi
