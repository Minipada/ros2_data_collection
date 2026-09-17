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
# Two modes (#533):
#
#   # per-PR gate (ci.yaml's verify-runtime-images): exactly what this run built
#   DC_ROS_IMAGE=ghcr.io/<repo>/dc-ros:<distro>-<sha> \
#   DC_UPLOADER_IMAGE=ghcr.io/<repo>/dc-uploader:<distro>-<sha> \
#     ./tools/release/scripts/verify_runtime_images.sh
#
#   # all-distros check (by hand, when wanted): no image env — derive every distro's
#   # floating {distro} refs and validate each pair
#   ./tools/release/scripts/verify_runtime_images.sh
#
# Env vars:
#   DC_ROS_IMAGE       with DC_UPLOADER_IMAGE, switches to single-pair mode (both or
#                      neither).
#   DC_UPLOADER_IMAGE  see DC_ROS_IMAGE.
#   DC_IMAGE_REGISTRY  (derive mode) registry/repo the floating refs are derived from;
#                      defaults to the upstream repo the manifests pin.
#   DC_DISTROS         (derive mode) distros to validate; default "jazzy lyrical rolling".
#   VECTOR_VERSION     Vector image tag; defaults to ros2_data_collection.repos'
#                      vector_vendor pin (#484: single source, lib/version.sh's
#                      resolver — the same one run_split.sh uses).
#   DC_RELEASE_TIMEOUT_SECONDS  deadline for the first Record to land (default 60).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELEASE_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(dirname "$(dirname "$RELEASE_DIR")")"
RUN_DIR="$RELEASE_DIR/.run"
mkdir -p "$RUN_DIR"

DC_IMAGE_REGISTRY="${DC_IMAGE_REGISTRY:-ghcr.io/minipada/ros2_data_collection}"
DC_DISTROS="${DC_DISTROS:-jazzy lyrical rolling}"
# The resolver lives with the e2e lib it was extracted from (#484) — this script already
# borrows tools/e2e fixtures (compose.test.yaml) rather than duplicating them.
# shellcheck disable=SC1091
source "$REPO_ROOT/tools/e2e/scripts/lib/version.sh"
VECTOR_VERSION="${VECTOR_VERSION:-$(vector_version)}"
VECTOR_IMAGE="docker.io/timberio/vector:${VECTOR_VERSION}-debian"
TIMEOUT_SECONDS="${DC_RELEASE_TIMEOUT_SECONDS:-60}"

# dc_pg_test is the fixed Postgres container name from tools/e2e/compose.test.yaml, not
# ours to choose; RustFS is only reached over 127.0.0.1:9000 below, never by name.
PG_C=dc_pg_test
DC_ROS_C=dc_release_smoke_dc_ros
UPLOADER_C=dc_release_smoke_dc_uploader
VECTOR_C=dc_release_smoke_vector
CONFIG_VOL=dc_release_smoke_config
BUFFER_VOL=dc_release_smoke_buffer
UPLOADER_VOL=dc_release_smoke_uploader

export PODMAN_COMPOSE_PROVIDER="${PODMAN_COMPOSE_PROVIDER:-podman-compose}"

teardown() {
  # The subshell's own exit status survives the EXIT trap; every command here is
  # best-effort so the trap can never mask a FAIL with a teardown error.
  local label="$1" run_dir="$2"
  log "$label" "tearing down"
  for c in "$DC_ROS_C" "$UPLOADER_C" "$VECTOR_C"; do
    podman logs "$c" > "$run_dir/${c}.log" 2>&1 || true
  done
  podman rm -f "$DC_ROS_C" "$UPLOADER_C" "$VECTOR_C" >/dev/null 2>&1 || true
  podman compose -f "$REPO_ROOT/tools/e2e/compose.test.yaml" down >/dev/null 2>&1 || true
  podman volume rm -f "$CONFIG_VOL" "$BUFFER_VOL" "$UPLOADER_VOL" >/dev/null 2>&1 || true
}

log() { echo "[verify-runtime-images $1 $(date -u +%H:%M:%S)] $*"; }

# One full pull-and-run smoke for a single dc-ros/dc-uploader pair, in a subshell: its
# EXIT trap tears the topology down whether the pair passed or failed, and a failure
# exits only this subshell — the all-distros loop below keeps going.
smoke() {
  local label="$1" dc_ros_image="$2" dc_uploader_image="$3"
  local run_dir="$RUN_DIR/$label"
  mkdir -p "$run_dir"
  (
    trap 'teardown "$label" "$run_dir"' EXIT

    pg_exec() { podman exec "$PG_C" psql -U dc -d dc -tAc "$1"; }

    # --- destinations: reuse the existing store-backed-test fixtures (compose.test.yaml)
    log "$label" "starting Postgres + RustFS"
    podman compose -f "$REPO_ROOT/tools/e2e/compose.test.yaml" up -d

    timeout 60 bash -c "until podman exec $PG_C pg_isready -U dc >/dev/null 2>&1; do sleep 1; done" \
      || { log "$label" "Postgres never became ready"; exit 1; }
    timeout 60 bash -c "until curl -sf http://127.0.0.1:9000 >/dev/null 2>&1 || curl -s http://127.0.0.1:9000 >/dev/null 2>&1; do sleep 1; done" \
      || { log "$label" "RustFS never became ready"; exit 1; }

    # compose.test.yaml's Postgres carries no schema (unlike compose.split.yaml's, which
    # mounts tools/e2e/sql/init.sql) — dc_bridge's postgres sink maps event keys onto
    # existing columns 1:1, it never creates the table. Just enough columns for the
    # `uptime` measurement smoke_params.yaml uses. Truncate so the count below starts
    # from zero even against a long-lived local fixture other runs have written to.
    pg_exec "CREATE TABLE IF NOT EXISTS dc_records (date bigint, tag text, group_key text, time double precision)"
    pg_exec "TRUNCATE dc_records"

    podman run --rm --network host \
      -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
      docker.io/amazon/aws-cli:latest \
      --endpoint-url http://127.0.0.1:9000 s3 mb s3://dc-release-smoke >/dev/null 2>&1 || true

    # --- pull the published images (no local build) -------------------------------------
    log "$label" "pulling published images (dc-ros=$dc_ros_image, dc-uploader=$dc_uploader_image, vector=$VECTOR_IMAGE)"
    podman pull "$dc_ros_image"
    podman pull "$dc_uploader_image"
    podman pull "$VECTOR_IMAGE"

    podman volume create "$CONFIG_VOL" >/dev/null
    podman volume create "$BUFFER_VOL" >/dev/null
    podman volume create "$UPLOADER_VOL" >/dev/null

    # --- vector: unmanaged Shipper, waits for dc-ros's atomically-written config --------
    log "$label" "starting vector"
    podman run -d --network host --name "$VECTOR_C" \
      -v "$BUFFER_VOL:/var/lib/vector" \
      -v "$CONFIG_VOL:/etc/dc/shipper:ro" \
      --entrypoint /bin/sh "$VECTOR_IMAGE" -c \
      'until [ -f /etc/dc/shipper/vector.toml ]; do sleep 0.2; done; exec vector --config /etc/dc/shipper/vector.toml --watch-config' \
      >/dev/null

    # --- dc-uploader: standalone, proves it starts cleanly off DC_UPLOADER_* env alone ----
    log "$label" "starting dc-uploader"
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
      "$dc_uploader_image" \
      >/dev/null

    # --- dc-ros: unmanaged-shipper mode, no in-process uploader ---------------------------
    log "$label" "starting dc-ros"
    podman run -d --network host --name "$DC_ROS_C" \
      -v "$CONFIG_VOL:/etc/dc/shipper" \
      -v "$RELEASE_DIR/params/smoke_params.yaml:/opt/dc/smoke_params.yaml:ro" \
      -v "$RELEASE_DIR/params/smoke_pgsql_sink.toml:/root/.dc/smoke_pgsql_sink.toml:ro" \
      "$dc_ros_image" \
      dc_params_file:=/opt/dc/smoke_params.yaml run_uploader:=false \
      >/dev/null

    log "$label" "waiting up to ${TIMEOUT_SECONDS}s for dc-ros to report ready"
    timeout "$TIMEOUT_SECONDS" bash -c "until podman logs $DC_ROS_C 2>&1 | grep -q 'dc_bridge reports ready'; do sleep 1; done" \
      || { log "$label" "FAIL: dc-ros never reported ready"; exit 1; }
    log "$label" "PASS: dc-ros reports ready"

    for c in "$VECTOR_C" "$UPLOADER_C"; do
      if [ "$(podman inspect --format '{{.State.Running}}' "$c")" != "true" ]; then
        log "$label" "FAIL: $c is not running"
        exit 1
      fi
    done
    log "$label" "PASS: vector and dc-uploader are both still running"

    log "$label" "waiting up to ${TIMEOUT_SECONDS}s for a Record to land in Postgres"
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
      log "$label" "FAIL: no Record landed in Postgres within ${TIMEOUT_SECONDS}s"
      exit 1
    fi
    log "$label" "PASS: dc-ros shipped a Record through vector into Postgres ($COUNT row(s))"

    log "$label" "PASS: published dc-ros/dc-uploader images run the three-container topology with no local build"
  )
}

if [ -n "${DC_ROS_IMAGE:-}" ] || [ -n "${DC_UPLOADER_IMAGE:-}" ]; then
  # Single-pair mode: the per-PR gate — both refs required, no derivation.
  : "${DC_ROS_IMAGE:?DC_ROS_IMAGE must be set to a published dc-ros image ref}"
  : "${DC_UPLOADER_IMAGE:?DC_UPLOADER_IMAGE must be set to a published dc-uploader image ref}"
  smoke custom "$DC_ROS_IMAGE" "$DC_UPLOADER_IMAGE"
  exit 0
fi

# Derive mode: every distro's floating ref, validated in turn. Errexit is suspended
# around each call so one broken distro is recorded and the rest still run; smoke()
# redeclares `set -euo pipefail` inside its own subshell, so a failure aborts only
# that distro.
FAILURES=()
read -r -a DISTRO_LIST <<< "$DC_DISTROS"
for DISTRO in "${DISTRO_LIST[@]}"; do
  log "$DISTRO" "=== validating $DC_IMAGE_REGISTRY/{dc-ros,dc-uploader}:$DISTRO"
  set +e
  smoke "$DISTRO" "$DC_IMAGE_REGISTRY/dc-ros:$DISTRO" "$DC_IMAGE_REGISTRY/dc-uploader:$DISTRO"
  RC=$?
  set -e
  if [ "$RC" -ne 0 ]; then
    log "$DISTRO" "FAIL: distro failed (rc=$RC), logs under $RUN_DIR/$DISTRO"
    FAILURES+=("$DISTRO")
  fi
done

log "summary" "validated: $DC_DISTROS"
if [ "${#FAILURES[@]}" -gt 0 ]; then
  log "summary" "FAIL: ${FAILURES[*]} not runnable as the three-container topology"
  exit 1
fi
log "summary" "PASS: every distro's published dc-ros/dc-uploader images run the three-container topology with no local build"
