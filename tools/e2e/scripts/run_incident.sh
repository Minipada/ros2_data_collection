#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Incident-capture E2E scenario (#291). From the repo root:
#
#   ./tools/e2e/scripts/run_incident.sh
#
# Reuses the same dc-e2e image tools/e2e/scripts/run.sh builds/uses (see its header for the
# build/prebuilt-image env vars this script shares: DC_E2E_IMAGE, DC_WORKSPACE_IMAGE), but
# drives the DC stack against tools/e2e/params/e2e_incident_params.yaml: an `uptime`
# Measurement armed for pre-event buffering (`buffer_duration_sec`) alongside a `memory`
# Measurement collecting live, both routed to one `postgres` Destination.
#
# Asserts, against a real Postgres container, the real dc_bridge and the real Vector binary:
#   1. while armed, the buffered Measurement ships nothing — its Records are held, not
#      published — while the live one keeps landing rows. (Without this the final query
#      could pass on a pipeline that simply never buffered anything.)
#   2. after one FlushEvent, the released window is queryable as
#      `WHERE incident_id = '<the id the event carried>'` — a *column* predicate. This is the
#      point of the whole scenario: the same rows are reachable by column, not by grepping
#      an opaque JSON payload, which is what #291 is for.
#   3. every released Record carries that exact id and the Tag of the armed Measurement, and
#      the live Measurement's rows keep `incident_id IS NULL` — the field marks an incident,
#      it is not stamped on everything.
#
# The FlushEvent is published directly onto /dc/flush with `ros2 topic pub` instead of being
# produced by a `dc_triggers` broadcast node. That topic *is* the contract a Measurement
# subscribes to (dc_triggers has its own tests for minting the event); wiring the broadcast
# node into dc_bringup's launch is separate work, and this scenario is about what happens to
# the id downstream of the event, not about what produced it.
#
# Env vars: DC_E2E_IMAGE / DC_WORKSPACE_IMAGE (see run.sh); DC_E2E_KEEP ("true" to leave the
# stack up after a failure for debugging); DC_E2E_LIVE_TIMEOUT_SECONDS (default 180 — how
# long to wait for the live Measurement's first rows, which is also how long the armed one is
# observed shipping nothing; generous because it covers the whole stack's startup, Vector
# included); DC_E2E_INCIDENT_TIMEOUT_SECONDS (default 120 — from the FlushEvent to the
# released window being committed, which includes the post-roll phase and Vector's own batch).
#
# The shared harness skeleton (image resolution, cleanup trap, Postgres bring-up and its
# init.sql-aware readiness wait) lives in lib/harness.sh. No RustFS here: both Measurements
# route to Postgres only. Every Record assertion lives in verify_zero_loss.py's `incident`
# profile (#496) — this script's own jobs are the topology, the FlushEvent, and the two
# bounded waits those assertions sit on the far side of.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run"

LIVE_TIMEOUT_SECONDS="${DC_E2E_LIVE_TIMEOUT_SECONDS:-180}"
INCIDENT_TIMEOUT_SECONDS="${DC_E2E_INCIDENT_TIMEOUT_SECONDS:-120}"

# Fixed rather than random: a failed run leaves rows in the Postgres volume that are worth
# being able to find again by name, and the whole point is that the id the event carried is
# the id the column holds — a generated one would prove the same thing while making a
# debugging session guess what to query.
INCIDENT_ID="e2e-incident-0001"
BUFFERED_TAG="dc.measurement.uptime"
LIVE_TAG="dc.measurement.memory"

PG_C=dc_e2e_inc_postgres
DC_C=dc_e2e_inc_dc
VOLUMES=(dc_e2e_inc_pgdata dc_e2e_inc_buffer dc_e2e_inc_data)

# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/harness.sh"

cd "$E2E_DIR"

harness_init \
  --tag e2e-incident \
  --run-dir "$RUN_DIR" \
  --network host \
  --containers "$DC_C" "$PG_C" \
  --volumes "${VOLUMES[*]}" \
  --log-captures "$DC_C:dc_incident.log" \
  --pg-container "$PG_C"

# --- obtain the DC stack image (shared with run.sh — see its header) ------------------
DC_IMAGE="" # set by resolve_image
resolve_image

remove_stack

# --- Postgres ------------------------------------------------------------------------
log "starting Postgres (sql/init.sql — dc_records has the incident_id column under test)"
start_postgres dc_e2e_inc_pgdata
wait_postgres_ready

# --- DC stack --------------------------------------------------------------------------
log "starting the DC stack (uptime armed with buffer_duration_sec, memory collecting live)"
podman run -d --network host --name "$DC_C" \
  -v dc_e2e_inc_buffer:/root/.dc/e2e/buffer \
  -v dc_e2e_inc_data:/root/.dc/e2e/data \
  -v "$E2E_DIR/params/e2e_incident_params.yaml:/opt/e2e/e2e_params.yaml:ro" \
  -v "$E2E_DIR/params/e2e_incident_pgsql_sink.toml:/opt/e2e/e2e_incident_pgsql_sink.toml:ro" \
  "$DC_IMAGE" >/dev/null

# --- 1. armed means silent, live means flowing -------------------------------------------
log "waiting up to ${LIVE_TIMEOUT_SECONDS}s for the live Measurement's first rows, then asserting the armed one shipped nothing"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --postgres-container "$PG_C" \
  --profile incident \
  --stage armed \
  --incident-id "$INCIDENT_ID" \
  --live-tag "$LIVE_TAG" \
  --buffered-tag "$BUFFERED_TAG" \
  --timeout-seconds "$LIVE_TIMEOUT_SECONDS" \
  --report "$RUN_DIR/verification_report_incident_armed.json"

# --- 2. one FlushEvent, then the window is queryable by column ---------------------------
log "publishing one FlushEvent on /dc/flush with incident_id='$INCIDENT_ID'"
# `-w 1`: the publisher lives in a throwaway process, and a volatile subscription that has
# not finished discovery yet simply never sees the sample — waiting for the Measurement's
# subscription to match removes that race instead of papering over it with repeats. Wrapped
# in `timeout` so a Measurement that never subscribed fails the run here, where the message
# says why, rather than 120s later as an empty result set.
if ! timeout 90 podman exec "$DC_C" bash -lc "
  set +u
  source /root/ws/install/setup.bash
  set -u
  ros2 topic pub --once -w 1 /dc/flush dc_interfaces/msg/FlushEvent '{incident_id: \"$INCIDENT_ID\"}'
" >/dev/null; then
  log "FAIL: could not publish a FlushEvent that a Measurement was subscribed to"
  exit 1
fi

# --- 3. the window is queryable by column, and by nothing else ----------------------------
# The assertion this whole scenario exists for: a column predicate, not a payload match.
log "waiting up to ${INCIDENT_TIMEOUT_SECONDS}s for the released window to reach Postgres"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --postgres-container "$PG_C" \
  --profile incident \
  --stage released \
  --incident-id "$INCIDENT_ID" \
  --live-tag "$LIVE_TAG" \
  --buffered-tag "$BUFFERED_TAG" \
  --timeout-seconds "$INCIDENT_TIMEOUT_SECONDS" \
  --report "$RUN_DIR/verification_report_incident_released.json"

log "PASS: incident_id E2E scenario (#291)"
