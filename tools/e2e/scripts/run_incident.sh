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
# route to Postgres only.
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
# shellcheck disable=SC2034  # consumed by lib/harness.sh
HARNESS_TAG=e2e-incident

# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/harness.sh"

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

remove_stack() {
  podman rm -f --ignore "$DC_C" "$PG_C" >/dev/null
  for v in "${VOLUMES[@]}"; do
    if podman volume exists "$v"; then
      podman volume rm "$v" >/dev/null
    fi
  done
}

# shellcheck disable=SC2034  # consumed by lib/harness.sh's teardown
HARNESS_LOG_CAPTURES=("$DC_C:dc_incident.log")
trap harness_cleanup EXIT

# --- obtain the DC stack image (shared with run.sh — see its header) ------------------
DC_IMAGE="" # set by resolve_image
resolve_image

remove_stack

# --- Postgres ------------------------------------------------------------------------
log "starting Postgres (sql/init.sql — dc_records has the incident_id column under test)"
start_postgres dc_e2e_inc_pgdata
wait_postgres_ready

# The column has to be a column. A table without it would make every assertion below fail
# for a reason that has nothing to do with the pipeline, so say so here instead.
COLUMN_TYPE="$(pg_exec "SELECT data_type FROM information_schema.columns WHERE table_name = 'dc_records' AND column_name = 'incident_id'" | tr -d '[:space:]')"
if [ "$COLUMN_TYPE" != "text" ]; then
  log "FAIL: dc_records has no text incident_id column (got '${COLUMN_TYPE:-none}') — check sql/init.sql"
  exit 1
fi
log "PASS: dc_records.incident_id exists as a text column"

# --- DC stack --------------------------------------------------------------------------
log "starting the DC stack (uptime armed with buffer_duration_sec, memory collecting live)"
podman run -d --network host --name "$DC_C" \
  -v dc_e2e_inc_buffer:/root/.dc/e2e/buffer \
  -v dc_e2e_inc_data:/root/.dc/e2e/data \
  -v "$E2E_DIR/params/e2e_incident_params.yaml:/opt/e2e/e2e_params.yaml:ro" \
  -v "$E2E_DIR/params/e2e_incident_pgsql_sink.toml:/opt/e2e/e2e_incident_pgsql_sink.toml:ro" \
  "$DC_IMAGE" >/dev/null

# A count query that reads 0 rather than aborting the run while Postgres/the table is still
# settling — every call site is inside a deadline loop that fails loudly on its own.
pg_count() {
  pg_exec "$1" 2>/dev/null | tr -d '[:space:]' || true
}

# --- 1. armed means silent, live means flowing -------------------------------------------
log "waiting up to ${LIVE_TIMEOUT_SECONDS}s for the live Measurement's first rows"
DEADLINE=$(( $(date +%s) + LIVE_TIMEOUT_SECONDS ))
LIVE_COUNT=0
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  LIVE_COUNT="$(pg_count "SELECT count(*) FROM dc_records WHERE tag = '$LIVE_TAG'")"
  # Several rows, not one: enough collection has gone by that an unarmed Measurement would
  # have shipped several Records too, which is what makes the armed one's silence meaningful.
  if [ "${LIVE_COUNT:-0}" -ge 5 ] 2>/dev/null; then
    break
  fi
  sleep 2
done

if [ "${LIVE_COUNT:-0}" -lt 5 ] 2>/dev/null; then
  log "FAIL: the live Measurement produced only ${LIVE_COUNT:-0} rows in ${LIVE_TIMEOUT_SECONDS}s — the pipeline is not running"
  exit 1
fi
log "PASS: ${LIVE_COUNT} live rows — the pipeline is delivering to Postgres"

BUFFERED_COUNT="$(pg_count "SELECT count(*) FROM dc_records WHERE tag = '$BUFFERED_TAG'")"
if [ "${BUFFERED_COUNT:-0}" -ne 0 ] 2>/dev/null; then
  log "FAIL: the armed Measurement shipped ${BUFFERED_COUNT} rows before any FlushEvent — it is not buffering"
  exit 1
fi
log "PASS: the armed Measurement shipped nothing while buffering"

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

log "waiting up to ${INCIDENT_TIMEOUT_SECONDS}s for the released window to reach Postgres"
DEADLINE=$(( $(date +%s) + INCIDENT_TIMEOUT_SECONDS ))
INCIDENT_COUNT=0
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  # The assertion this whole scenario exists for: a column predicate, not a payload match.
  INCIDENT_COUNT="$(pg_count "SELECT count(*) FROM dc_records WHERE incident_id = '$INCIDENT_ID'")"
  if [ "${INCIDENT_COUNT:-0}" -ge 2 ] 2>/dev/null; then
    break
  fi
  sleep 2
done

if [ "${INCIDENT_COUNT:-0}" -lt 2 ] 2>/dev/null; then
  log "FAIL: only ${INCIDENT_COUNT:-0} row(s) matched WHERE incident_id = '$INCIDENT_ID' within ${INCIDENT_TIMEOUT_SECONDS}s"
  log "      (a released *window* is several Records; 0 means the id never became a column value at all)"
  exit 1
fi
log "PASS: ${INCIDENT_COUNT} rows are queryable as WHERE incident_id = '$INCIDENT_ID' — a column, not payload text"

# --- 3. the id marks the incident and nothing else ----------------------------------------
WRONG_TAG="$(pg_count "SELECT count(*) FROM dc_records WHERE incident_id = '$INCIDENT_ID' AND tag <> '$BUFFERED_TAG'")"
if [ "${WRONG_TAG:-0}" -ne 0 ] 2>/dev/null; then
  log "FAIL: ${WRONG_TAG} incident row(s) came from a Measurement other than $BUFFERED_TAG"
  exit 1
fi

OTHER_IDS="$(pg_count "SELECT count(*) FROM dc_records WHERE incident_id IS NOT NULL AND incident_id <> '$INCIDENT_ID'")"
if [ "${OTHER_IDS:-0}" -ne 0 ] 2>/dev/null; then
  log "FAIL: ${OTHER_IDS} row(s) carry an incident_id other than the one the FlushEvent minted"
  exit 1
fi

LIVE_TAGGED="$(pg_count "SELECT count(*) FROM dc_records WHERE tag = '$LIVE_TAG' AND incident_id IS NOT NULL")"
if [ "${LIVE_TAGGED:-0}" -ne 0 ] 2>/dev/null; then
  log "FAIL: ${LIVE_TAGGED} row(s) from the never-armed Measurement carry an incident_id"
  exit 1
fi
log "PASS: the incident_id is on the released window only — the live Measurement's rows stay NULL"

log "PASS: incident_id E2E scenario (#291)"
