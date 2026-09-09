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

PG_C=dc_e2e_ceiling_postgres
RUSTFS_C=dc_e2e_ceiling_rustfs
DC_C=dc_e2e_ceiling_dc
VOLUMES=(dc_e2e_ceiling_pgdata dc_e2e_ceiling_rustfs_data dc_e2e_ceiling_buffer dc_e2e_ceiling_data)

cd "$E2E_DIR"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/harness.sh"

harness_init \
  --tag e2e-ceiling \
  --run-dir "$RUN_DIR" \
  --network host \
  --containers "$DC_C" "$PG_C" "$RUSTFS_C" \
  --volumes "${VOLUMES[*]}" \
  --log-captures "$DC_C:dc.log" \
  --pg-container "$PG_C" \
  --rustfs-container "$RUSTFS_C"

# --- obtain the DC stack image (same convention as run.sh — see its header) -----------
DC_IMAGE="" # set by resolve_image
resolve_image

remove_stack

# --- destinations + the one real DC stack ----------------------------------------------
log "starting Postgres + RustFS"
start_postgres dc_e2e_ceiling_pgdata
start_rustfs dc_e2e_ceiling_rustfs_data
# 60, as before: the ramp's own level timing starts from the stack being up.
wait_postgres_ready 60
wait_rustfs_ready

log "creating the RustFS bucket"
create_rustfs_bucket http://127.0.0.1:9000

log "starting the DC stack"
podman run -d --network host --name "$DC_C" \
  -v dc_e2e_ceiling_buffer:/root/.dc/e2e/buffer \
  -v dc_e2e_ceiling_data:/root/.dc/e2e/data \
  "$DC_IMAGE" >/dev/null

log "waiting for the first Record to reach Postgres"
wait_first_record 60 "$(date +%s.%N)" "of starting the stack" >/dev/null
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
extract_from_volume dc_e2e_ceiling_data bash \
  -c 'cat /vol/passthrough/records.ndjson 2>/dev/null || true' > "$RUN_DIR/passthrough.ndjson"

log "extracting raw mode's Destination output"
extract_from_volume dc_e2e_ceiling_data bash \
  -c 'cat /vol/raw/records.ndjson 2>/dev/null || true' > "$RUN_DIR/raw.ndjson"

log "summarizing the MCAP passthrough writer's output"
extract_from_volume dc_e2e_ceiling_data python3 /opt/e2e/mcap_summary.py /vol/mcap \
  > "$RUN_DIR/mcap_summary.json"

log "extracting the workload ledger"
extract_from_volume dc_e2e_ceiling_data bash \
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
