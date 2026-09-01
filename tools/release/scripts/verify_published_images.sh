#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Proves the published `dc-ros` and `dc-uploader` images (#448) run the three-container
# split topology (dc-ros + vector + dc-uploader) with zero local `podman build` — only
# `podman pull` and `podman run`, against the public upstream Vector image. This is a
# boot-and-ship smoke check, not the zero-loss proof: that's
# tools/e2e/scripts/run_split.sh, an existing, separate seam (#440's Testing Decisions)
# that still runs against a locally-built dc-e2e image with its own workload generator.
# Here, a single Record making it from dc-ros through vector into Postgres is enough to
# show the published images themselves are runnable, not just CI-internal build
# artifacts.
#
#   DC_ROS_IMAGE=ghcr.io/<repo>/dc-ros:<sha> \
#   DC_UPLOADER_IMAGE=ghcr.io/<repo>/dc-uploader:<sha> \
#     ./tools/release/scripts/verify_published_images.sh
#
# Env vars:
#   DC_ROS_IMAGE       (required) the dc-ros image ref to pull and run.
#   DC_UPLOADER_IMAGE  (required) the dc-uploader image ref to pull and run.
#   VECTOR_VERSION     Vector image tag; defaults to ros2_data_collection.repos'
#                      vector_vendor pin (#448: single source, same grep run_split.sh uses).
#   DC_RELEASE_TIMEOUT_SECONDS  deadline for the first Record to land (default 60).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELEASE_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(dirname "$(dirname "$RELEASE_DIR")")"
RUN_DIR="$RELEASE_DIR/.run"
mkdir -p "$RUN_DIR"

DC_ROS_IMAGE="${DC_ROS_IMAGE:?DC_ROS_IMAGE must be set to a published dc-ros image ref}"
DC_UPLOADER_IMAGE="${DC_UPLOADER_IMAGE:?DC_UPLOADER_IMAGE must be set to a published dc-uploader image ref}"
VECTOR_VERSION="${VECTOR_VERSION:-$(grep -oP 'version: v\K[0-9.]+' "$REPO_ROOT/ros2_data_collection.repos")}"
VECTOR_IMAGE="docker.io/timberio/vector:${VECTOR_VERSION}-debian"
TIMEOUT_SECONDS="${DC_RELEASE_TIMEOUT_SECONDS:-60}"

log() { echo "[verify-published-images $(date -u +%H:%M:%S)] $*"; }

# dc_pg_test is the fixed Postgres container name from tools/e2e/compose.test.yaml, not
# ours to choose; RustFS is only reached over 127.0.0.1:9000 below, never by name.
PG_C=dc_pg_test
DC_ROS_C=dc_release_smoke_dc_ros
UPLOADER_C=dc_release_smoke_dc_uploader
VECTOR_C=dc_release_smoke_vector
CONFIG_VOL=dc_release_smoke_config
BUFFER_VOL=dc_release_smoke_buffer
UPLOADER_VOL=dc_release_smoke_uploader

pg_exec() { podman exec "$PG_C" psql -U dc -d dc -tAc "$1"; }

cleanup() {
  local exit_code=$?
  log "tearing down"
  for c in "$DC_ROS_C" "$UPLOADER_C" "$VECTOR_C"; do
    podman logs "$c" > "$RUN_DIR/${c}.log" 2>&1 || true
  done
  podman rm -f "$DC_ROS_C" "$UPLOADER_C" "$VECTOR_C" >/dev/null 2>&1 || true
  podman compose -f "$REPO_ROOT/tools/e2e/compose.test.yaml" down >/dev/null 2>&1 || true
  podman volume rm -f "$CONFIG_VOL" "$BUFFER_VOL" "$UPLOADER_VOL" >/dev/null 2>&1 || true
  exit "$exit_code"
}
trap cleanup EXIT

# --- destinations: reuse the existing store-backed-test fixtures (compose.test.yaml) --
log "starting Postgres + RustFS"
export PODMAN_COMPOSE_PROVIDER="${PODMAN_COMPOSE_PROVIDER:-podman-compose}"
podman compose -f "$REPO_ROOT/tools/e2e/compose.test.yaml" up -d

timeout 60 bash -c "until podman exec $PG_C pg_isready -U dc >/dev/null 2>&1; do sleep 1; done" \
  || { log "Postgres never became ready"; exit 1; }
timeout 60 bash -c "until curl -sf http://127.0.0.1:9000 >/dev/null 2>&1 || curl -s http://127.0.0.1:9000 >/dev/null 2>&1; do sleep 1; done" \
  || { log "RustFS never became ready"; exit 1; }

# compose.test.yaml's Postgres carries no schema (unlike compose.split.yaml's, which
# mounts tools/e2e/sql/init.sql) — dc_bridge's postgres sink maps event keys onto
# existing columns 1:1, it never creates the table. Just enough columns for the
# `uptime` measurement smoke_params.yaml uses.
pg_exec "CREATE TABLE IF NOT EXISTS dc_records (date bigint, tag text, group_key text, time double precision)"

podman run --rm --network host \
  -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
  docker.io/amazon/aws-cli:latest \
  --endpoint-url http://127.0.0.1:9000 s3 mb s3://dc-release-smoke >/dev/null 2>&1 || true

# --- pull the published images (no local build) ---------------------------------------
log "pulling published images (dc-ros=$DC_ROS_IMAGE, dc-uploader=$DC_UPLOADER_IMAGE, vector=$VECTOR_IMAGE)"
podman pull "$DC_ROS_IMAGE"
podman pull "$DC_UPLOADER_IMAGE"
podman pull "$VECTOR_IMAGE"

podman volume create "$CONFIG_VOL" >/dev/null
podman volume create "$BUFFER_VOL" >/dev/null
podman volume create "$UPLOADER_VOL" >/dev/null

# --- vector: unmanaged Shipper, waits for dc-ros's atomically-written config ----------
log "starting vector"
podman run -d --network host --name "$VECTOR_C" \
  -v "$BUFFER_VOL:/var/lib/vector" \
  -v "$CONFIG_VOL:/etc/dc/shipper:ro" \
  --entrypoint /bin/sh "$VECTOR_IMAGE" -c \
  'until [ -f /etc/dc/shipper/vector.toml ]; do sleep 0.2; done; exec vector --config /etc/dc/shipper/vector.toml --watch-config' \
  >/dev/null

# --- dc-uploader: standalone, proves it starts cleanly off DC_UPLOADER_* env alone ----
log "starting dc-uploader"
podman run -d --network host --name "$UPLOADER_C" \
  -v "$UPLOADER_VOL:/root/.dc/smoke/uploader" \
  -e DC_UPLOADER_STORAGE_NAME=rustfs \
  -e DC_UPLOADER_QUEUE_DIR=/root/.dc/smoke/uploader/queue/upload \
  -e DC_UPLOADER_STATE_DIR=/root/.dc/smoke/uploader/uploader \
  -e DC_UPLOADER_SHIPPER_HOST=127.0.0.1 \
  -e DC_UPLOADER_SHIPPER_PORT=24224 \
  -e DC_UPLOADER_S3_BUCKET=dc-release-smoke \
  -e DC_UPLOADER_S3_ENDPOINT=http://127.0.0.1:9000 \
  -e DC_UPLOADER_S3_ACCESS_KEY_ID=rustfsadmin \
  -e DC_UPLOADER_S3_SECRET_ACCESS_KEY=rustfsadmin \
  -e DC_UPLOADER_S3_FORCE_PATH_STYLE=true \
  "$DC_UPLOADER_IMAGE" \
  >/dev/null

# --- dc-ros: unmanaged-shipper mode, no in-process uploader ---------------------------
log "starting dc-ros"
podman run -d --network host --name "$DC_ROS_C" \
  -v "$CONFIG_VOL:/etc/dc/shipper" \
  -v "$RELEASE_DIR/params/smoke_params.yaml:/opt/dc/smoke_params.yaml:ro" \
  "$DC_ROS_IMAGE" \
  dc_params_file:=/opt/dc/smoke_params.yaml run_uploader:=false \
  >/dev/null

log "waiting up to ${TIMEOUT_SECONDS}s for dc-ros to report ready"
timeout "$TIMEOUT_SECONDS" bash -c "until podman logs $DC_ROS_C 2>&1 | grep -q 'dc_bridge reports ready'; do sleep 1; done" \
  || { log "FAIL: dc-ros never reported ready"; exit 1; }
log "PASS: dc-ros reports ready"

for c in "$VECTOR_C" "$UPLOADER_C"; do
  if [ "$(podman inspect --format '{{.State.Running}}' "$c")" != "true" ]; then
    log "FAIL: $c is not running"
    exit 1
  fi
done
log "PASS: vector and dc-uploader are both still running"

log "waiting up to ${TIMEOUT_SECONDS}s for a Record to land in Postgres"
DEADLINE=$(( $(date +%s) + TIMEOUT_SECONDS ))
COUNT=0
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  COUNT="$(pg_exec 'SELECT count(*) FROM dc_records' 2>/dev/null || echo 0)"
  if [ "${COUNT:-0}" -gt 0 ] 2>/dev/null; then
    break
  fi
  sleep 1
done
if [ "${COUNT:-0}" -eq 0 ]; then
  log "FAIL: no Record landed in Postgres within ${TIMEOUT_SECONDS}s"
  exit 1
fi
log "PASS: dc-ros shipped a Record through vector into Postgres ($COUNT row(s))"

log "PASS: published dc-ros/dc-uploader images run the three-container topology with no local build"
