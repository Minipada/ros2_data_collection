# shellcheck shell=bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Shared skeleton of the five e2e scenario scripts, run{,_split,_degraded,_incident,
# _retention}.sh (#483): obtaining the DC image, the cleanup trap, destination bring-up
# and readiness waits, volume extraction. Sourced, never executed — set -euo pipefail
# comes from the caller.
#
# Contract with the sourcing script (set before the first call):
#   PG_C, RUSTFS_C       destination container names (start_*/wait_*/pg_exec)
#   RUN_DIR              teardown log captures land here
#   remove_stack()       the caller's teardown — each scenario owns its own container,
#                        volume (and, for run_split.sh, compose) cleanup
#   HARNESS_TAG          log-line prefix (default "e2e")
#   HARNESS_NETWORK      network for destinations, probes and the bucket CLI
#                        (default "host")
#   HARNESS_LOG_CAPTURES "container:file" pairs whose podman logs land in RUN_DIR on
#                        teardown
#   STATS_PID            optional resource-sampler PID killed on teardown

HARNESS_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HARNESS_E2E_DIR="$(dirname "$HARNESS_SCRIPT_DIR")"

# rustfs/rustfs 1.0.0-beta.11 — pinned by digest, not :latest, so an upstream image
# change can't silently alter harness behavior. Bump deliberately: check
# https://hub.docker.com/r/rustfs/rustfs/tags for the new digest.
RUSTFS_IMAGE="docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c"

log() { echo "[${HARNESS_TAG:-e2e} $(date -u +%H:%M:%S)] $*"; }

# Sets DC_IMAGE: DC_E2E_IMAGE (prebuilt, no build) > DC_WORKSPACE_IMAGE (prebuilt base)
# > a local build.sh + Containerfile.e2e. CI's e2e jobs set DC_E2E_IMAGE to the :<sha>
# image their build job pushed, so the harness runs the exact built artifact.
resolve_image() {
  if [ -n "${DC_E2E_IMAGE:-}" ]; then
    log "using prebuilt E2E image: $DC_E2E_IMAGE (no build)"
    podman image exists "$DC_E2E_IMAGE" || podman pull "$DC_E2E_IMAGE"
    DC_IMAGE="$DC_E2E_IMAGE"
  else
    local workspace_image
    if [ -n "${DC_WORKSPACE_IMAGE:-}" ]; then
      log "using prebuilt DC workspace image: $DC_WORKSPACE_IMAGE (skipping build.sh)"
      podman image exists "$DC_WORKSPACE_IMAGE" || podman pull "$DC_WORKSPACE_IMAGE"
      workspace_image="$DC_WORKSPACE_IMAGE"
    else
      log "building the DC workspace image (tools/e2e/scripts/build.sh — the same build CI uses)"
      "$HARNESS_SCRIPT_DIR/build.sh"
      workspace_image="dc-workspace:latest"
    fi
    log "building the E2E image (Containerfile.e2e, FROM the workspace image)"
    podman build --build-arg "BASE_IMAGE=$workspace_image" -t dc-e2e:latest \
      -f "$HARNESS_E2E_DIR/Containerfile.e2e" "$HARNESS_E2E_DIR"
    DC_IMAGE="dc-e2e:latest"
  fi
}

# The EXIT trap: kill a running resource sampler, honour DC_E2E_KEEP, capture the
# HARNESS_LOG_CAPTURES logs, then run the caller's remove_stack. Install with
# `trap harness_cleanup EXIT`.
harness_cleanup() {
  local exit_code=$?
  if [ -n "${STATS_PID:-}" ] && kill -0 "$STATS_PID" 2>/dev/null; then
    kill "$STATS_PID"
  fi
  if [ "$exit_code" -ne 0 ] && [ "${DC_E2E_KEEP:-false}" = "true" ]; then
    log "FAILED (exit $exit_code) — leaving the stack up (DC_E2E_KEEP=true) for debugging"
    exit "$exit_code"
  fi
  log "tearing down"
  local pair container
  for pair in ${HARNESS_LOG_CAPTURES[@]+"${HARNESS_LOG_CAPTURES[@]}"}; do
    container="${pair%%:*}"
    if podman container exists "$container"; then
      podman logs "$container" > "$RUN_DIR/${pair##*:}" 2>&1 || true
    fi
  done
  remove_stack
  exit "$exit_code"
}

pg_exec() {
  podman exec "$PG_C" psql -U dc -d dc -tAc "$1"
}

# start_postgres <pgdata-volume>: the destination Postgres, with sql/init.sql applied
# on the volume's first boot.
start_postgres() {
  podman run -d --network "${HARNESS_NETWORK:-host}" --name "$PG_C" \
    -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
    -v "$1:/var/lib/postgresql/data" \
    -v "$HARNESS_E2E_DIR/sql/init.sql:/docker-entrypoint-initdb.d/init.sql:ro" \
    docker.io/library/postgres:13 >/dev/null
}

start_rustfs() {
  podman run -d --network "${HARNESS_NETWORK:-host}" --name "$RUSTFS_C" \
    -v "$1:/data" \
    "$RUSTFS_IMAGE" >/dev/null
}

# Deliberately not pg_isready: the postgres image applies /docker-entrypoint-initdb.d
# against a *temporary* server that pg_isready happily answers, racing init.sql. Waiting
# for dc_records to resolve gates on the only state that matters — the real server is up
# and init.sql applied.
wait_postgres_ready() {
  local timeout="${1:-120}"
  timeout "$timeout" bash -c "until podman exec $PG_C psql -U dc -d dc -tAc \"SELECT to_regclass('public.dc_records')\" 2>/dev/null | grep -q dc_records; do sleep 2; done" \
    || { log "FAIL: Postgres never came up with sql/init.sql applied"; exit 1; }
}

# On the host network RustFS is probed directly; otherwise from a one-off container on
# HARNESS_NETWORK. That probe needs --entrypoint python3: the image's own ENTRYPOINT is
# entrypoint.sh (the full DC stack), so without the override the probe args land as
# extra ros2-launch arguments and the container always exits non-zero regardless of
# RustFS's actual reachability.
wait_rustfs_ready() {
  local timeout="${1:-60}"
  if [ "${HARNESS_NETWORK:-host}" = "host" ]; then
    timeout "$timeout" bash -c 'until curl -sf http://127.0.0.1:9000 >/dev/null 2>&1 || curl -s http://127.0.0.1:9000 >/dev/null 2>&1; do sleep 1; done' \
      || { log "FAIL: RustFS never became ready"; exit 1; }
  else
    log "waiting for RustFS to accept TCP connections on $HARNESS_NETWORK"
    timeout "$timeout" bash -c "until podman run --rm --network $HARNESS_NETWORK --entrypoint python3 '$DC_IMAGE' /opt/e2e/measure_rtt.py $RUSTFS_C 9000 --count 1 --timeout 2 >/dev/null 2>&1; do sleep 1; done" \
      || { log "FAIL: RustFS never became reachable on $HARNESS_NETWORK"; exit 1; }
  fi
}

# The AWS CLI talks plain S3 to RustFS via --endpoint-url — same protocol dc_bridge's
# Uploader uses (aws-sdk-cpp), so no extra vendor in the loop. Neither the S3 sink nor
# the Uploader creates the bucket; someone has to.
create_rustfs_bucket() {
  podman run --rm --network "${HARNESS_NETWORK:-host}" \
    -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
    docker.io/amazon/aws-cli:latest \
    --endpoint-url "$1" s3 mb s3://dc-e2e
}

# extract_from_volume <volume> <entrypoint> [args...]: a one-off read-only container on
# <volume> (mounted at /vol) from the DC image; stdout passes through to the caller's
# redirect or command substitution.
extract_from_volume() {
  local volume="$1" entrypoint="$2"
  shift 2
  podman run --rm --entrypoint "$entrypoint" -v "$volume:/vol:ro" "$DC_IMAGE" "$@"
}
