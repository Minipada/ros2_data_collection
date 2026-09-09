#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Files retention E2E scenario (#267). From the repo root:
#
#   ./tools/e2e/scripts/run_retention.sh
#
# Reuses the same dc-e2e image tools/e2e/scripts/run.sh builds/uses (see its own header
# for the build/prebuilt-image env vars this script shares: DC_E2E_IMAGE,
# DC_WORKSPACE_IMAGE), but drives the DC stack against
# tools/e2e/params/e2e_retention_params.yaml instead of the zero-loss harness's own
# params — a small `files.retention.max_bytes` and RustFS deliberately never started
# until told to, so the un-uploaded File pool the camera Measurement feeds is guaranteed
# to exceed the limit purely from the store being unreachable (ADR-0005's #267 scenario:
# "store down, publish Files past a small max_bytes").
#
# Asserts, against real Postgres/RustFS containers and the real dc_bridge binary:
#   1. with RustFS down, an audit row (`deleted: true, uploaded: false` — the "shed
#      without upload" signature) lands in `dc_files` for the oldest camera capture,
#      once the pool exceeds max_bytes.
#   2. once RustFS comes up, a later (unshed) camera capture uploads and verifies
#      normally (`uploaded: true` row in `dc_files`, object present in the bucket).
#
# Env vars: DC_E2E_IMAGE / DC_WORKSPACE_IMAGE (see run.sh); DC_E2E_KEEP ("true" to leave
# the stack up after a failure for debugging); DC_E2E_SHED_TIMEOUT_SECONDS (default 480
# — genuinely needed, not just generous padding: the camera Measurement's 15s poll and
# the workload generator's 15s publish period aren't phase-locked, so a capture can take
# a couple of poll cycles to land; and with RustFS down, aws-sdk-cpp's own
# connection-refused retry/backoff makes a single Uploader attempt take on the order of
# a minute — verified empirically in this environment, same pre-existing characteristic
# #265's own notes call out — not something this scenario's timeout should race against);
# DC_E2E_UPLOAD_TIMEOUT_SECONDS (default 60, once RustFS is up connections are fast).
#
# The shared harness skeleton (image resolution, cleanup trap, destination bring-up and
# readiness waits) lives in lib/harness.sh. The two dc_files assertions live in
# verify_zero_loss.py's `retention` profile (#496); what stays here is the topology (a
# store that only comes up halfway through), the shed File's absence *on disk*, and the
# bounded waits those assertions sit on the far side of.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run"

SHED_TIMEOUT_SECONDS="${DC_E2E_SHED_TIMEOUT_SECONDS:-480}"
UPLOAD_TIMEOUT_SECONDS="${DC_E2E_UPLOAD_TIMEOUT_SECONDS:-60}"

PG_C=dc_e2e_ret_postgres
RUSTFS_C=dc_e2e_ret_rustfs
DC_C=dc_e2e_ret_dc
VOLUMES=(dc_e2e_ret_pgdata dc_e2e_ret_rustfs_data dc_e2e_ret_buffer dc_e2e_ret_data)

# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/harness.sh"

cd "$E2E_DIR"

harness_init \
  --tag e2e-retention \
  --run-dir "$RUN_DIR" \
  --network host \
  --containers "$DC_C" "$PG_C" "$RUSTFS_C" \
  --volumes "${VOLUMES[*]}" \
  --log-captures "$DC_C:dc_retention.log" \
  --pg-container "$PG_C" \
  --rustfs-container "$RUSTFS_C"

# --- obtain the DC stack image (shared with run.sh — see its header) ----------------
DC_IMAGE="" # set by resolve_image
resolve_image

remove_stack

# --- bring up Postgres only — RustFS deliberately stays down ------------------------
log "starting Postgres (RustFS deliberately NOT started yet — this scenario's 'store down')"
start_postgres dc_e2e_ret_pgdata
wait_postgres_ready

# --- start the DC stack against the retention params ---------------------------------
log "starting the DC stack (files.retention.max_bytes configured small; RustFS unreachable)"
podman run -d --network host --name "$DC_C" \
  -v dc_e2e_ret_buffer:/root/.dc/e2e/buffer \
  -v dc_e2e_ret_data:/root/.dc/e2e/data \
  -v "$E2E_DIR/params/e2e_retention_params.yaml:/opt/e2e/e2e_params.yaml:ro" \
  -v "$E2E_DIR/params/e2e_retention_pgsql_sink.toml:/opt/e2e/e2e_retention_pgsql_sink.toml:ro" \
  "$DC_IMAGE" >/dev/null

# --- verify the shed happens while the store is down ---------------------------------
log "waiting up to ${SHED_TIMEOUT_SECONDS}s for a shed audit row (deleted=true, uploaded=false) in dc_files"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --postgres-container "$PG_C" \
  --profile retention \
  --stage shed \
  --timeout-seconds "$SHED_TIMEOUT_SECONDS" \
  --report "$RUN_DIR/verification_report_retention_shed.json"
log "PASS: the oldest un-uploaded File(s) were shed while RustFS was down"

# The shed local File must actually be gone (File+intent atomicity) — not just the row.
SHED_LOCAL_PATH="$(pg_exec "SELECT local_path FROM dc_files WHERE deleted = true AND uploaded = false ORDER BY updated_at ASC LIMIT 1" 2>/dev/null || true)"
if [ -n "$SHED_LOCAL_PATH" ]; then
  # Same mount path the DC container itself uses (not a fresh /vol) so the absolute
  # local_path Postgres recorded resolves directly, no prefix rewriting needed.
  if podman run --rm --entrypoint bash -v dc_e2e_ret_data:/root/.dc/e2e/data:ro "$DC_IMAGE" \
       -c "[ ! -e '$SHED_LOCAL_PATH' ]"; then
    log "PASS: the shed File's local path no longer exists on disk"
  else
    log "FAIL: shed audit row exists but the local File is still on disk: $SHED_LOCAL_PATH"
    exit 1
  fi
fi

# --- bring RustFS up; remaining (unshed) Files must upload normally ------------------
log "starting RustFS — remaining/future Files must now upload and verify normally"
start_rustfs dc_e2e_ret_rustfs_data
wait_rustfs_ready
create_rustfs_bucket http://127.0.0.1:9000 >/dev/null

log "waiting up to ${UPLOAD_TIMEOUT_SECONDS}s for a normal upload (uploaded=true) in dc_files now that RustFS is up"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --postgres-container "$PG_C" \
  --profile retention \
  --stage uploaded \
  --timeout-seconds "$UPLOAD_TIMEOUT_SECONDS" \
  --report "$RUN_DIR/verification_report_retention_uploaded.json"
log "PASS: File(s) uploaded and verified normally once the store came back"

log "PASS: files retention E2E scenario (#267)"
