#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Limits harness: single-robot ceiling axis (#383, part of #323's epic). From the repo
# root:
#
#   ./tools/e2e/scripts/run_limits_single_robot_ceiling.sh
#
# Brings up, with plain podman (a sibling of run.sh/run_limits_two_tier.sh — same
# Postgres+RustFS destinations, one real DC stack, `--network host` since there's only
# ever one stack here, matching run.sh rather than the two-tier composer's dedicated
# bridge network), the topology this axis measures against, then hands off to
# run_limits_single_robot_ceiling.py to ramp the *real* workload generator's per-topic
# publish rate — through real Measurements, the real Bridge, and the stack's own local
# Shipper — until saturation_probe.py's verdict trips, reusing ramp_controller.py's
# find_knee() and saturation_probe.py's evaluate() completely unchanged (#323's PRD:
# "the ramp controller and saturation probe are reused unchanged; only the driver
# adapter and topology wiring are new"). Only the driver adapter
# (scripts/single_robot_ceiling_driver.py) and this topology wiring are axis-specific.
#
# Zero loss is asserted once, cumulatively, after the ramp finishes and a drain window
# lets any still-in-flight backlog land — see run_limits_single_robot_ceiling.py's own
# header for why not after every individual level. verify_zero_loss.py (#249's
# unmodified verifier) is reused exactly as run.sh/run_limits_two_tier.sh call it.
#
# Env vars (all optional):
#   DC_E2E_CEILING_LEVELS            comma-separated ascending Records/s levels to ramp
#                                     through (default 1,2,5,10,20,40,80)
#   DC_E2E_CEILING_LEVEL_DURATION    seconds driven at each level (default 30)
#   DC_E2E_CEILING_SAMPLE_INTERVAL   seconds between polls within a level (default 3;
#                                     must yield at least saturation_probe.py's
#                                     min_observations, 4, per level: level_duration /
#                                     sample_interval >= 4)
#   DC_E2E_CEILING_TOPIC             the ledger-checked synth topic to ramp (default synth00)
#   DC_E2E_CEILING_DRAIN_SECONDS     settle time after the ramp, before verifying (default 30)
#   DC_E2E_KEEP                      "true" to leave the stack up after a failure for debugging
#   DC_E2E_IMAGE / DC_WORKSPACE_IMAGE   same meaning as run.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run/limits_single_robot_ceiling"

LEVELS="${DC_E2E_CEILING_LEVELS:-1,2,5,10,20,40,80}"
LEVEL_DURATION="${DC_E2E_CEILING_LEVEL_DURATION:-30}"
SAMPLE_INTERVAL="${DC_E2E_CEILING_SAMPLE_INTERVAL:-3}"
TOPIC="${DC_E2E_CEILING_TOPIC:-synth00}"
DRAIN_SECONDS="${DC_E2E_CEILING_DRAIN_SECONDS:-30}"
KEEP="${DC_E2E_KEEP:-false}"

PG_C=dc_e2e_ceiling_postgres
RUSTFS_C=dc_e2e_ceiling_rustfs
DC_C=dc_e2e_ceiling_dc
VOLUMES=(dc_e2e_ceiling_pgdata dc_e2e_ceiling_rustfs_data dc_e2e_ceiling_buffer dc_e2e_ceiling_data)

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

log() { echo "[e2e-ceiling $(date -u +%H:%M:%S)] $*"; }

remove_stack() {
  podman rm -f --ignore "$DC_C" "$PG_C" "$RUSTFS_C" >/dev/null
  local v
  for v in "${VOLUMES[@]}"; do
    if podman volume exists "$v"; then
      podman volume rm "$v" >/dev/null
    fi
  done
}

cleanup() {
  local exit_code=$?
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

# --- obtain the DC stack image (same convention as run.sh — see its header) -----------
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

remove_stack

# --- destinations + the one real DC stack ----------------------------------------------
log "starting Postgres + RustFS"
podman run -d --network host --name "$PG_C" \
  -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
  -v dc_e2e_ceiling_pgdata:/var/lib/postgresql/data \
  -v "$E2E_DIR/sql/init.sql:/docker-entrypoint-initdb.d/init.sql:ro" \
  docker.io/library/postgres:13 >/dev/null
podman run -d --network host --name "$RUSTFS_C" \
  -v dc_e2e_ceiling_rustfs_data:/data \
  docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c >/dev/null

timeout 60 bash -c "until podman exec $PG_C pg_isready -U dc >/dev/null 2>&1; do sleep 1; done" \
  || { log "FAIL: Postgres never became ready"; exit 1; }
timeout 60 bash -c 'until curl -sf http://127.0.0.1:9000 >/dev/null 2>&1 || curl -s http://127.0.0.1:9000 >/dev/null 2>&1; do sleep 1; done' \
  || { log "FAIL: RustFS never became ready"; exit 1; }

log "creating the RustFS bucket"
podman run --rm --network host \
  -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
  docker.io/amazon/aws-cli:latest \
  --endpoint-url http://127.0.0.1:9000 s3 mb s3://dc-e2e

log "starting the DC stack"
podman run -d --network host --name "$DC_C" \
  -v dc_e2e_ceiling_buffer:/root/.dc/e2e/buffer \
  -v dc_e2e_ceiling_data:/root/.dc/e2e/data \
  "$DC_IMAGE" >/dev/null

log "waiting for the first Record to reach Postgres"
timeout 60 bash -c "until [ \"\$(podman exec $PG_C psql -U dc -d dc -tAc 'SELECT count(*) FROM dc_records' 2>/dev/null || echo 0)\" -gt 0 ] 2>/dev/null; do sleep 1; done" \
  || { log "FAIL: no Record landed in Postgres within 60s of starting the stack"; exit 1; }
log "PASS: the stack is up and publishing"

# --- ramp the single-robot ceiling axis, via the *unmodified* ramp controller/probe ---
log "ramping levels [$LEVELS] Records/s on topic $TOPIC, ${LEVEL_DURATION}s/level, ${SAMPLE_INTERVAL}s polling"
uv run --frozen python3 "$SCRIPT_DIR/run_limits_single_robot_ceiling.py" \
  --dc-container "$DC_C" \
  --postgres-container "$PG_C" \
  --topic "$TOPIC" \
  --levels "$LEVELS" \
  --level-duration "$LEVEL_DURATION" \
  --sample-interval "$SAMPLE_INTERVAL" \
  --report-json "$RUN_DIR/curve_report.json" \
  --report-txt "$RUN_DIR/curve_report.txt"

log "draining for ${DRAIN_SECONDS}s (lets any backlog from the ramp's final level land)"
sleep "$DRAIN_SECONDS"

log "stopping the DC stack so counts settle before verification"
podman stop "$DC_C" >/dev/null
sleep 5

# --- extract, then verify zero-loss with the existing, unmodified verifier -----------
log "extracting the passthrough sink's output"
podman run --rm --entrypoint bash \
  -v dc_e2e_ceiling_data:/vol:ro "$DC_IMAGE" \
  -c 'cat /vol/passthrough/records.ndjson 2>/dev/null || true' > "$RUN_DIR/passthrough.ndjson"

log "extracting raw mode's Destination output"
podman run --rm --entrypoint bash \
  -v dc_e2e_ceiling_data:/vol:ro "$DC_IMAGE" \
  -c 'cat /vol/raw/records.ndjson 2>/dev/null || true' > "$RUN_DIR/raw.ndjson"

log "summarizing the MCAP passthrough writer's output"
podman run --rm --entrypoint python3 \
  -v dc_e2e_ceiling_data:/vol:ro "$DC_IMAGE" \
  /opt/e2e/mcap_summary.py /vol/mcap > "$RUN_DIR/mcap_summary.json"

log "extracting the workload ledger"
podman run --rm --entrypoint bash \
  -v dc_e2e_ceiling_data:/vol:ro "$DC_IMAGE" \
  -c 'cat /vol/workload_ledger.txt 2>/dev/null || true' > "$RUN_DIR/workload_ledger.txt"

log "verifying zero-loss across the whole ramp"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --postgres-container "$PG_C" \
  --num-synth-topics 14 \
  --ledger-file "$RUN_DIR/workload_ledger.txt" \
  --passthrough-file "$RUN_DIR/passthrough.ndjson" \
  --mcap-summary-file "$RUN_DIR/mcap_summary.json" \
  --raw-file "$RUN_DIR/raw.ndjson" \
  --report "$RUN_DIR/verification_report.json"

log "PASS: single-robot ceiling axis (#383) — see $RUN_DIR/curve_report.txt"
cat "$RUN_DIR/curve_report.txt"
