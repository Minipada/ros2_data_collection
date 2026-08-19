#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Limits harness: Uploader concurrency axis (#384, part of #323's epic). From the repo
# root:
#
#   ./tools/e2e/scripts/run_limits_upload_concurrency.sh
#
# Stands up a shared Postgres + RustFS (the one object-storage Destination this axis
# ramps concurrent Uploaders against) on a dedicated podman bridge network, then hands
# off to tools/e2e/scripts/run_upload_concurrency_axis.py, which owns the ramp itself:
# it calls tools/e2e/scripts/ramp_controller.py's find_knee() unchanged, driving load by
# starting N Bridge/Uploader containers per level (the Uploader has no internal
# concurrency knob — see that script's own header for why "concurrency" means process
# count) against params/e2e_limits_upload_params.yaml, and feeds the result to
# tools/e2e/scripts/curve_reporter.py's build_report().
#
# Per #384's acceptance criteria, file sizes stay the deliberately synthetic/reduced-scale
# 64x64 solid-color image workload_generator.py already produces for every E2E scenario
# (~12KB) — the claim under test is concurrency and custody (delete-after-verify
# ordering, Group completion correctness), not byte volume, and any byte-volume
# implication of the reported ceiling is arithmetic, never presented as measured.
#
# Env vars (all optional):
#   DC_E2E_UPLOAD_LEVELS              comma-separated ascending concurrency levels
#                                      (default "1,2,4,8" — kept modest per #323's PRD's
#                                      own dev-machine note: 8 real DC stacks concurrently
#                                      starting is already a meaningful load)
#   DC_E2E_UPLOAD_CAMERA_PERIOD_S     capture cadence fed to each container via
#                                      DC_E2E_CAMERA_PERIOD_S (default 2 — see
#                                      workload_generator.py and the params file's own
#                                      header for why the default 15s cadence is too slow
#                                      to build a queue-depth/backlog trend inside a short
#                                      steady-state window)
#   DC_E2E_UPLOAD_STEADY_STATE_SECONDS  per-level sampling window (default 60 — covers
#                                      workload_generator.py's own fixed ~30s
#                                      subscription-wait fallback before its first
#                                      publish, verified empirically, plus enough
#                                      real capture cycles afterward at
#                                      DC_E2E_UPLOAD_CAMERA_PERIOD_S to form a trend)
#   DC_E2E_UPLOAD_SAMPLE_INTERVAL_SECONDS  seconds between observations (default 5)
#   DC_E2E_KEEP                       "true" to leave the stack (and its network) up
#                                      after a failure for debugging
#   DC_E2E_IMAGE / DC_WORKSPACE_IMAGE   same meaning as run.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run/upload_concurrency"

LEVELS="${DC_E2E_UPLOAD_LEVELS:-1,2,4,8}"
CAMERA_PERIOD_S="${DC_E2E_UPLOAD_CAMERA_PERIOD_S:-2}"
STEADY_STATE_SECONDS="${DC_E2E_UPLOAD_STEADY_STATE_SECONDS:-60}"
SAMPLE_INTERVAL_SECONDS="${DC_E2E_UPLOAD_SAMPLE_INTERVAL_SECONDS:-5}"
KEEP="${DC_E2E_KEEP:-false}"

# Hyphens, not underscores — same reasoning as run_limits_two_tier.sh's PG_C/RUSTFS_C:
# these names are embedded in URLs (RustFS/Postgres endpoints), and aws-cli 2.36's
# --endpoint-url parser rejects a hostname with an underscore outright.
NET=dc-e2e-upload-net
PG_C=dc-e2e-upload-postgres
RUSTFS_C=dc-e2e-upload-rustfs

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

log() { echo "[e2e-upload-axis $(date -u +%H:%M:%S)] $*"; }

remove_stack() {
  # Any per-level Bridge containers/volumes are already cleaned up by
  # run_upload_concurrency_axis.py's own driver as each level finishes — only the
  # shared storage + network this script itself started remain here.
  podman rm -f --ignore "$PG_C" "$RUSTFS_C" >/dev/null
  for v in dc_e2e_upload_pgdata dc_e2e_upload_rustfs_data; do
    if podman volume exists "$v"; then
      podman volume rm "$v" >/dev/null
    fi
  done
  podman network rm "$NET" >/dev/null 2>&1 || true
}

cleanup() {
  local exit_code=$?
  if [ "$exit_code" -ne 0 ] && [ "$KEEP" = "true" ]; then
    log "FAILED (exit $exit_code) — leaving the shared storage up (DC_E2E_KEEP=true) for debugging"
    exit "$exit_code"
  fi
  log "tearing down shared storage"
  if podman container exists "$PG_C"; then
    podman logs "$PG_C" > "$RUN_DIR/postgres.log" 2>&1
  fi
  if podman container exists "$RUSTFS_C"; then
    podman logs "$RUSTFS_C" > "$RUN_DIR/rustfs.log" 2>&1
  fi
  remove_stack
  exit "$exit_code"
}
trap cleanup EXIT

# --- obtain the DC stack image (shared with run.sh — see its header) ------------------
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

# Clean any leftovers from a previous (possibly DC_E2E_KEEP=true) run.
remove_stack

# --- bridge network + the one shared object-storage Destination this axis ramps -------
log "creating the bridge network ($NET)"
podman network create "$NET" >/dev/null

log "starting the shared Postgres + RustFS on $NET"
podman run -d --network "$NET" --name "$PG_C" \
  -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
  -v dc_e2e_upload_pgdata:/var/lib/postgresql/data \
  -v "$E2E_DIR/sql/init.sql:/docker-entrypoint-initdb.d/init.sql:ro" \
  docker.io/library/postgres:13 >/dev/null
podman run -d --network "$NET" --name "$RUSTFS_C" \
  -v dc_e2e_upload_rustfs_data:/data \
  docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c >/dev/null

timeout 120 bash -c "until podman exec $PG_C psql -U dc -d dc -tAc \"SELECT to_regclass('public.dc_files')\" 2>/dev/null | grep -q dc_files; do sleep 2; done" \
  || { log "FAIL: Postgres never came up with sql/init.sql applied"; exit 1; }

log "waiting for RustFS to accept TCP connections on $NET"
# --entrypoint bash: the image's own ENTRYPOINT (entrypoint.sh) launches the full DC
# stack and appends any extra args to `ros2 launch` rather than running them as a
# separate command, so a one-off diagnostic run needs an explicit override — same
# reasoning and probe run_limits_two_tier.sh uses.
timeout 60 bash -c "
  until podman run --rm --network $NET --entrypoint bash '$DC_IMAGE' -c 'exec 3<>/dev/tcp/$RUSTFS_C/9000' >/dev/null 2>&1; do
    sleep 1
  done
" || { log "FAIL: RustFS never became reachable on $NET"; exit 1; }

log "creating the RustFS bucket"
podman run --rm --network "$NET" \
  -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
  docker.io/amazon/aws-cli:latest \
  --endpoint-url "http://$RUSTFS_C:9000" s3 mb s3://dc-e2e

# --- the ramp itself: run_upload_concurrency_axis.py owns everything from here --------
log "running the Uploader concurrency ramp (levels: $LEVELS)"
uv run --frozen python3 "$SCRIPT_DIR/run_upload_concurrency_axis.py" \
  --image "$DC_IMAGE" \
  --network "$NET" \
  --postgres-container "$PG_C" \
  --params-file "$E2E_DIR/params/e2e_limits_upload_params.yaml" \
  --run-dir "$RUN_DIR" \
  --levels "$LEVELS" \
  --camera-period-s "$CAMERA_PERIOD_S" \
  --steady-state-seconds "$STEADY_STATE_SECONDS" \
  --sample-interval-seconds "$SAMPLE_INTERVAL_SECONDS" \
  --report "$RUN_DIR/curve_report.json"

log "PASS: Uploader concurrency axis (#384)"
