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
#   DC_E2E_SPLIT_QUEUE_DRAIN_TIMEOUT_SECONDS bound for the upload intent queue to drain
#                                          after the drain window, before the check
#                                          fails (default 120 — the uploader's
#                                          exponential retry backoff can outlast
#                                          DC_E2E_SPLIT_DRAIN_SECONDS; see the check)
#   DC_E2E_KEEP                            "true" to leave the stack up on failure
#   DC_E2E_IMAGE / DC_WORKSPACE_IMAGE      same meaning as run.sh
#
# The shared harness skeleton (lib/harness.sh) is parameterized for this scenario by
# HARNESS_NETWORK=dc_e2e_split_net: readiness probes and the bucket CLI run as one-off
# containers on the compose network rather than the host.
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
QUEUE_DRAIN_TIMEOUT_SECONDS="${DC_E2E_SPLIT_QUEUE_DRAIN_TIMEOUT_SECONDS:-120}"

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
# shellcheck disable=SC2034  # consumed by lib/harness.sh
HARNESS_TAG=e2e-split
# shellcheck disable=SC2034
HARNESS_NETWORK=dc_e2e_split_net

# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/harness.sh"

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

compose() { podman compose -f "$COMPOSE_FILE" "$@"; }

remove_stack() {
  compose down --volumes --remove-orphans >/dev/null 2>&1 || true
}

# shellcheck disable=SC2034  # consumed by lib/harness.sh's teardown
HARNESS_LOG_CAPTURES=("$DC_ROS_C:dc-ros.log" "$UPLOADER_C:dc-uploader.log" "$VECTOR_C:vector.log")
trap harness_cleanup EXIT

# --- obtain the DC stack image (same logic as run.sh) --------------------------------
DC_IMAGE="" # set by resolve_image
resolve_image
export DC_E2E_IMAGE="$DC_IMAGE"

remove_stack

# --- bring up the destinations --------------------------------------------------------
log "starting Postgres + RustFS"
compose up -d postgres rustfs

wait_postgres_ready
wait_rustfs_ready

log "creating the RustFS bucket (the S3 sink doesn't create it)"
# The compose service name, not $RUSTFS_C: aws-cli's own --endpoint-url validation
# rejects underscores as an invalid hostname, and $RUSTFS_C (the container_name) has
# them — the compose service name ("rustfs") resolves the same way on
# dc_e2e_split_net and has none. dc-uploader's DC_UPLOADER_S3_ENDPOINT and
# e2e_split_params.yaml's rustfs.endpoint already use the service name for the same
# reason.
create_rustfs_bucket "http://rustfs:9000"

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
wait_postgres_ready
# RustFS too, same probe as bring-up above — not just Postgres: a Postgres-only gate
# lets the uploader's first post-restore attempt hit a still-warming object store,
# fail, and double that intent's retry backoff (doubling is what pushes it past the
# fixed drain window below).
wait_rustfs_ready

log "draining for ${DRAIN_SECONDS}s"
sleep "$DRAIN_SECONDS"

log "stopping the workload so counts settle before verification"
podman stop "$DC_ROS_C" >/dev/null
sleep 5

# --- durable upload intent queue (#265) — same standard as run.sh ---------------------
# The uploader retries a failed intent on an exponential backoff (5s base, doubling,
# capped high — dc_bridge's intent_queue.hpp). In this split topology the uploader
# process rides out the whole outage un-restarted, so intents that failed against the
# dead RustFS can legitimately still be inside that backoff when the fixed drain
# window ends — unlike run.sh, where the mid-outage restart of the all-in-one dc
# container resets it. So: wait for the queue to drain with dc-uploader still running
# (a stopped uploader can't drain anything), bounded — an intent that never drains
# within the bound is a real orphan and still fails the run, per #265.
log "verifying the durable upload intent queue drained (no orphaned intents after the outage+restart)"
QUEUE_DEADLINE=$(( $(date +%s) + QUEUE_DRAIN_TIMEOUT_SECONDS ))
while :; do
  LEFTOVER_INTENTS=$(extract_from_volume dc_e2e_split_uploader bash \
    -c 'find /vol/queue/upload -maxdepth 1 -name "*.json" 2>/dev/null | wc -l')
  if [ "${LEFTOVER_INTENTS:-0}" -eq 0 ]; then
    break
  fi
  if [ "$(date +%s)" -ge "$QUEUE_DEADLINE" ]; then
    log "FAIL: ${LEFTOVER_INTENTS} orphaned upload intent(s) left in the queue after the run"
    exit 1
  fi
  sleep 5
done
log "PASS: upload intent queue empty (0 orphaned intents)"

log "stopping dc-uploader and vector (queue drained, nothing left in flight)"
podman stop "$UPLOADER_C" "$VECTOR_C" >/dev/null

# --- verify -----------------------------------------------------------------------
log "extracting the workload ledger (what the generator published)"
extract_from_volume dc_e2e_split_data bash \
  -c 'cat /vol/workload_ledger.txt 2>/dev/null || true' > "$RUN_DIR/workload_ledger_split.txt"

log "verifying zero-loss against the published ledger (no passthrough/MCAP/raw checks — see params/e2e_split_params.yaml's header)"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --postgres-container "$PG_C" \
  --num-synth-topics 14 \
  --ledger-file "$RUN_DIR/workload_ledger_split.txt" \
  --report "$RUN_DIR/verification_report_split.json"

log "PASS: split-topology zero-loss E2E scenario"
