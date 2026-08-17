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
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run"

SHED_TIMEOUT_SECONDS="${DC_E2E_SHED_TIMEOUT_SECONDS:-480}"
UPLOAD_TIMEOUT_SECONDS="${DC_E2E_UPLOAD_TIMEOUT_SECONDS:-60}"
KEEP="${DC_E2E_KEEP:-false}"

PG_C=dc_e2e_ret_postgres
RUSTFS_C=dc_e2e_ret_rustfs
DC_C=dc_e2e_ret_dc
VOLUMES=(dc_e2e_ret_pgdata dc_e2e_ret_rustfs_data dc_e2e_ret_buffer dc_e2e_ret_data)

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

log() { echo "[e2e-retention $(date -u +%H:%M:%S)] $*"; }

remove_stack() {
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

cleanup() {
  local exit_code=$?
  if [ "$exit_code" -ne 0 ] && [ "$KEEP" = "true" ]; then
    log "FAILED (exit $exit_code) — leaving the stack up (DC_E2E_KEEP=true) for debugging"
    exit "$exit_code"
  fi
  log "tearing down"
  if podman container exists "$DC_C"; then
    podman logs "$DC_C" > "$RUN_DIR/dc_retention.log" 2>&1
  fi
  remove_stack
  exit "$exit_code"
}
trap cleanup EXIT

# --- obtain the DC stack image (shared with run.sh — see its header) ----------------
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

# --- bring up Postgres only — RustFS deliberately stays down ------------------------
log "starting Postgres (RustFS deliberately NOT started yet — this scenario's 'store down')"
podman run -d --network host --name "$PG_C" \
  -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
  -v dc_e2e_ret_pgdata:/var/lib/postgresql/data \
  -v "$E2E_DIR/sql/init.sql:/docker-entrypoint-initdb.d/init.sql:ro" \
  docker.io/library/postgres:13 >/dev/null

timeout 60 bash -c "until podman exec $PG_C pg_isready -U dc >/dev/null 2>&1; do sleep 1; done" \
  || { log "Postgres never became ready"; exit 1; }

# --- start the DC stack against the retention params ---------------------------------
log "starting the DC stack (files.retention.max_bytes configured small; RustFS unreachable)"
podman run -d --network host --name "$DC_C" \
  -v dc_e2e_ret_buffer:/root/.dc/e2e/buffer \
  -v dc_e2e_ret_data:/root/.dc/e2e/data \
  -v "$E2E_DIR/params/e2e_retention_params.yaml:/opt/e2e/e2e_params.yaml:ro" \
  "$DC_IMAGE" >/dev/null

# --- verify the shed happens while the store is down ---------------------------------
log "waiting up to ${SHED_TIMEOUT_SECONDS}s for a shed audit row (deleted=true, uploaded=false) in dc_files"
DEADLINE=$(( $(date +%s) + SHED_TIMEOUT_SECONDS ))
SHED_COUNT=0
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  SHED_COUNT="$(pg_exec "SELECT count(*) FROM dc_files WHERE deleted = true AND uploaded = false" 2>/dev/null || echo 0)"
  if [ "${SHED_COUNT:-0}" -gt 0 ] 2>/dev/null; then
    break
  fi
  sleep 2
done

if [ "${SHED_COUNT:-0}" -eq 0 ]; then
  log "FAIL: no shed-without-upload audit row landed within ${SHED_TIMEOUT_SECONDS}s"
  exit 1
fi
log "PASS: ${SHED_COUNT} shed-without-upload audit row(s) — the oldest un-uploaded File(s) were shed while RustFS was down"

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
podman run -d --network host --name "$RUSTFS_C" \
  -v dc_e2e_ret_rustfs_data:/data \
  docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c >/dev/null
timeout 60 bash -c 'until curl -sf http://127.0.0.1:9000 >/dev/null 2>&1 || curl -s http://127.0.0.1:9000 >/dev/null 2>&1; do sleep 1; done' \
  || { log "RustFS never became ready"; exit 1; }
podman run --rm --network host \
  -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
  docker.io/amazon/aws-cli:latest \
  --endpoint-url http://127.0.0.1:9000 s3 mb s3://dc-e2e >/dev/null

log "waiting up to ${UPLOAD_TIMEOUT_SECONDS}s for a normal upload (uploaded=true) in dc_files now that RustFS is up"
DEADLINE=$(( $(date +%s) + UPLOAD_TIMEOUT_SECONDS ))
UPLOADED_COUNT=0
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  UPLOADED_COUNT="$(pg_exec "SELECT count(*) FROM dc_files WHERE uploaded = true" 2>/dev/null || echo 0)"
  if [ "${UPLOADED_COUNT:-0}" -gt 0 ] 2>/dev/null; then
    break
  fi
  sleep 2
done

if [ "${UPLOADED_COUNT:-0}" -eq 0 ]; then
  log "FAIL: no File uploaded/verified within ${UPLOAD_TIMEOUT_SECONDS}s of RustFS coming up"
  exit 1
fi
log "PASS: ${UPLOADED_COUNT} File(s) uploaded and verified normally once the store came back"

log "PASS: files retention E2E scenario (#267)"
