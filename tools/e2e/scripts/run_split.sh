#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Split-topology E2E scenario (#445/#447, part of #440): the same zero-loss guarantee
# run.sh proves for the all-in-one mode, but with dc-ros, vector and dc-uploader as
# separate Compose-managed containers (compose.split.yaml). See tools/e2e/README.md for
# what this proves.
#
#   ./tools/e2e/scripts/run_split.sh
#
# Env vars (all optional):
#   DC_E2E_SPLIT_VECTOR_DELAY_SECONDS      how long dc-ros runs before vector starts
#                                          (default 20, under bridge_ready_gate's 120s)
#   DC_E2E_SPLIT_RECOVERY_TIMEOUT_SECONDS  deadline for the first Record once vector
#                                          starts (default 60)
#   DC_E2E_SPLIT_STEADY_STATE_SECONDS      warmup before the outage (default 15)
#   DC_E2E_SPLIT_OUTAGE_SECONDS            outage duration (default 60)
#   DC_E2E_SPLIT_DRAIN_SECONDS             settle time before verifying (default 30)
#   DC_E2E_KEEP                            "true" to leave the stack up on failure
#   DC_E2E_IMAGE / DC_WORKSPACE_IMAGE      same meaning as run.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run"
COMPOSE_FILE="$E2E_DIR/compose.split.yaml"

VECTOR_DELAY_SECONDS="${DC_E2E_SPLIT_VECTOR_DELAY_SECONDS:-20}"
RECOVERY_TIMEOUT_SECONDS="${DC_E2E_SPLIT_RECOVERY_TIMEOUT_SECONDS:-60}"
STEADY_STATE_SECONDS="${DC_E2E_SPLIT_STEADY_STATE_SECONDS:-15}"
OUTAGE_SECONDS="${DC_E2E_SPLIT_OUTAGE_SECONDS:-60}"
DRAIN_SECONDS="${DC_E2E_SPLIT_DRAIN_SECONDS:-30}"
KEEP="${DC_E2E_KEEP:-false}"

# podman-compose, not the docker-compose cli-plugin (CLAUDE.md "Containers: Podman, not
# Docker"; matches ci.yaml's own compose.test.yaml usage) — it needs no Podman API socket.
export PODMAN_COMPOSE_PROVIDER="${PODMAN_COMPOSE_PROVIDER:-podman-compose}"

# compose.split.yaml's vector service interpolates this (#448: the Vector image tag and
# vector_vendor's pinned version come from one place) — same grep already used by
# run_limits_two_tier.sh / run_load_driver_shipper_test.sh / run_limits_drain_rate.sh /
# run_limits_shipper_fanin.sh, so the apt and container paths cannot drift apart.
VECTOR_VERSION="$(grep -A3 'vector_vendor:' "$E2E_DIR/../../ros2_data_collection.repos" | grep -oP 'version: v\K[0-9.]+')"
export VECTOR_VERSION

PG_C=dc_e2e_split_postgres
RUSTFS_C=dc_e2e_split_rustfs
DC_ROS_C=dc_e2e_split_dc_ros
UPLOADER_C=dc_e2e_split_dc_uploader
VECTOR_C=dc_e2e_split_vector

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

log() { echo "[e2e-split $(date -u +%H:%M:%S)] $*"; }

compose() { podman compose -f "$COMPOSE_FILE" "$@"; }

remove_stack() {
  compose down --volumes --remove-orphans >/dev/null 2>&1 || true
}

pg_exec() {
  podman exec "$PG_C" psql -U dc -d dc -tAc "$1"
}

cleanup() {
  local exit_code=$?
  if [ "$exit_code" -ne 0 ] && [ "$KEEP" = "true" ]; then
    log "FAILED (exit $exit_code) — leaving the stack up (DC_E2E_KEEP=true) for debugging"
    exit "$exit_code"
  fi
  log "tearing down"
  podman logs "$DC_ROS_C" > "$RUN_DIR/dc-ros.log" 2>&1 || true
  podman logs "$UPLOADER_C" > "$RUN_DIR/dc-uploader.log" 2>&1 || true
  podman logs "$VECTOR_C" > "$RUN_DIR/vector.log" 2>&1 || true
  remove_stack
  exit "$exit_code"
}
trap cleanup EXIT

# --- obtain the DC stack image (same logic as run.sh) --------------------------------
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
    log "building the DC workspace image (tools/e2e/scripts/build.sh)"
    "$SCRIPT_DIR/build.sh"
    WORKSPACE_IMAGE="dc-workspace:latest"
  fi
  log "building the E2E image (Containerfile.e2e, FROM the workspace image)"
  podman build --build-arg "BASE_IMAGE=$WORKSPACE_IMAGE" -t dc-e2e:latest -f Containerfile.e2e .
  DC_IMAGE="dc-e2e:latest"
fi
export DC_E2E_IMAGE="$DC_IMAGE"

remove_stack

# --- bring up the destinations --------------------------------------------------------
log "starting Postgres + RustFS"
compose up -d postgres rustfs

timeout 60 bash -c "until podman exec $PG_C pg_isready -U dc >/dev/null 2>&1; do sleep 1; done" \
  || { log "Postgres never became ready"; exit 1; }
# No --network host here (unlike run.sh) — probe from a container on dc_e2e_split_net,
# same as run_degraded.sh. --entrypoint python3 is required: $DC_IMAGE's own ENTRYPOINT
# is entrypoint.sh (the full DC stack), so without overriding it the probe args land as
# extra ros2-launch arguments instead of actually running measure_rtt.py — the container
# would always exit non-zero regardless of RustFS's actual reachability.
log "waiting for RustFS to accept TCP connections on dc_e2e_split_net"
timeout 60 bash -c "until podman run --rm --network dc_e2e_split_net --entrypoint python3 '$DC_IMAGE' \
  /opt/e2e/measure_rtt.py $RUSTFS_C 9000 --count 1 --timeout 2 >/dev/null 2>&1; do sleep 1; done" \
  || { log "RustFS never became reachable on dc_e2e_split_net"; exit 1; }

log "creating the RustFS bucket (the S3 sink doesn't create it)"
# The compose service name, not $RUSTFS_C: aws-cli's own --endpoint-url validation
# rejects underscores as an invalid hostname, and $RUSTFS_C (the container_name) has
# them — the compose service name ("rustfs") resolves the same way on
# dc_e2e_split_net and has none. dc-uploader's DC_UPLOADER_S3_ENDPOINT and
# e2e_split_params.yaml's rustfs.endpoint already use the service name for the same
# reason.
podman run --rm --network dc_e2e_split_net \
  -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
  docker.io/amazon/aws-cli:latest \
  --endpoint-url "http://rustfs:9000" s3 mb s3://dc-e2e

# --- start dc-ros + dc-uploader, prove no orchestrator-level ordering is needed ------
# vector deliberately isn't up yet: no depends_on in compose.split.yaml. dc-uploader
# comes up alongside dc-ros for the same reason — no depends_on between them either;
# IntentQueue::rescan() (#446) is what lets dc-uploader notice intents dc-ros writes
# after it has already started, and what lets dc-ros write intents dc-uploader hasn't
# started reading yet.
log "starting dc-ros + dc-uploader (vector is not up yet — proving no orchestrator-level ordering is required)"
compose up -d dc-ros dc-uploader
sleep "$VECTOR_DELAY_SECONDS"

log "starting vector (the Shipper), ${VECTOR_DELAY_SECONDS}s after dc-ros — measuring recovery"
VECTOR_START_TS=$(date +%s.%N)
compose up -d vector

FIRST_RECORD_LATENCY=""
DEADLINE=$(echo "$VECTOR_START_TS + $RECOVERY_TIMEOUT_SECONDS" | bc)
while (( $(echo "$(date +%s.%N) < $DEADLINE" | bc) )); do
  COUNT="$(pg_exec 'SELECT count(*) FROM dc_records' 2>/dev/null || echo 0)"
  if [ "${COUNT:-0}" -gt 0 ] 2>/dev/null; then
    FIRST_RECORD_LATENCY=$(echo "$(date +%s.%N) - $VECTOR_START_TS" | bc)
    break
  fi
  sleep 0.2
done

if [ -z "$FIRST_RECORD_LATENCY" ]; then
  log "FAIL: no Record landed in Postgres within ${RECOVERY_TIMEOUT_SECONDS}s of starting vector — dc-ros did not recover from the Shipper starting late"
  exit 1
fi
log "PASS: dc-ros recovered on its own (first Record ${FIRST_RECORD_LATENCY}s after vector started, no restart needed)"

# dc-uploader's own independent-restart proof (#447's per-container restart
# requirement) lives here, in steady state before the outage — not layered onto the
# outage/dc-ros-restart window below, which is timed against the zero-loss ledger
# comparison. Restarting two containers back to back was found to add enough jitter
# to the DDS-to-Shipper handoff to occasionally cost a Record outside the harness's
# per-restart kill-point tolerance (unrelated to dc_uploader itself, which shares no
# process or address space with dc-ros/vector — see ADR-0014); proving the restart
# here instead avoids that without weakening the guarantee it's proving.
log "restarting dc-uploader on its own, in steady state (proves it doesn't need dc-ros or vector restarted alongside it)"
podman restart "$UPLOADER_C" >/dev/null

log "steady state for ${STEADY_STATE_SECONDS}s"
sleep "$STEADY_STATE_SECONDS"

# --- outage -> restart -> restore, same standard as run.sh ---------------------------
log "inducing outage: stopping Postgres + RustFS for ${OUTAGE_SECONDS}s (dc-ros/vector keep running and buffering to disk — ADR-0002)"
podman stop "$PG_C" "$RUSTFS_C" >/dev/null
sleep "$OUTAGE_SECONDS"

log "restarting dc-ros while destinations are still down (proves recovery doesn't depend on dc-ros having stayed up, nor on vector or dc-uploader restarting alongside it)"
podman restart "$DC_ROS_C" >/dev/null

log "restoring Postgres + RustFS"
podman start "$PG_C" "$RUSTFS_C" >/dev/null
timeout 60 bash -c "until podman exec $PG_C pg_isready -U dc >/dev/null 2>&1; do sleep 1; done" \
  || { log "Postgres never came back"; exit 1; }

log "draining for ${DRAIN_SECONDS}s"
sleep "$DRAIN_SECONDS"

log "stopping the workload so counts settle before verification"
podman stop "$DC_ROS_C" "$UPLOADER_C" "$VECTOR_C" >/dev/null
sleep 5

# --- durable upload intent queue (#265) — same check as run.sh -----------------------
log "verifying the durable upload intent queue drained (no orphaned intents after the outage+restart)"
LEFTOVER_INTENTS=$(podman run --rm --entrypoint bash \
  -v dc_e2e_split_uploader:/vol:ro "$DC_IMAGE" \
  -c 'find /vol/queue/upload -maxdepth 1 -name "*.json" 2>/dev/null | wc -l')
if [ "${LEFTOVER_INTENTS:-0}" -ne 0 ]; then
  log "FAIL: ${LEFTOVER_INTENTS} orphaned upload intent(s) left in the queue after the run"
  exit 1
fi
log "PASS: upload intent queue empty (0 orphaned intents)"

# --- verify -----------------------------------------------------------------------
log "extracting the workload ledger (what the generator published)"
podman run --rm --entrypoint bash \
  -v dc_e2e_split_data:/vol:ro "$DC_IMAGE" \
  -c 'cat /vol/workload_ledger.txt 2>/dev/null || true' > "$RUN_DIR/workload_ledger_split.txt"

log "verifying zero-loss against the published ledger (no passthrough/MCAP/raw checks — see params/e2e_split_params.yaml's header)"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --postgres-container "$PG_C" \
  --num-synth-topics 14 \
  --ledger-file "$RUN_DIR/workload_ledger_split.txt" \
  --report "$RUN_DIR/verification_report_split.json"

log "PASS: split-topology zero-loss E2E scenario"
