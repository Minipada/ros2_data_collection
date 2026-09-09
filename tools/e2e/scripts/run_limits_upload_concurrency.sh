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

# Hyphens, not underscores — same reasoning as run_limits_two_tier.sh's PG_C/RUSTFS_C:
# these names are embedded in URLs (RustFS/Postgres endpoints), and aws-cli 2.36's
# --endpoint-url parser rejects a hostname with an underscore outright.
NET=dc-e2e-upload-net
PG_C=dc-e2e-upload-postgres
RUSTFS_C=dc-e2e-upload-rustfs

cd "$E2E_DIR"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/harness.sh"

# The per-level Bridge containers/volumes are run_upload_concurrency_axis.py's own driver's
# to clean up as each level finishes — only the shared storage this script starts is ours.
harness_init \
  --tag e2e-upload-axis \
  --run-dir "$RUN_DIR" \
  --network "$NET" \
  --containers "$PG_C" "$RUSTFS_C" \
  --volumes dc_e2e_upload_pgdata dc_e2e_upload_rustfs_data \
  --teardown-networks "$NET" \
  --log-captures "$PG_C:postgres.log" "$RUSTFS_C:rustfs.log" \
  --pg-container "$PG_C" \
  --rustfs-container "$RUSTFS_C"

# --- obtain the DC stack image (shared with run.sh — see its header) ------------------
DC_IMAGE="" # set by resolve_image
resolve_image

# Clean any leftovers from a previous (possibly DC_E2E_KEEP=true) run.
remove_stack

# --- bridge network + the one shared object-storage Destination this axis ramps -------
log "creating the bridge network ($NET)"
podman network create "$NET" >/dev/null

log "starting the shared Postgres + RustFS on $NET"
start_postgres dc_e2e_upload_pgdata
start_rustfs dc_e2e_upload_rustfs_data
# One pass of sql/init.sql creates dc_records and dc_files together, so gating on either
# gates on the other — this axis's ramp only writes dc_files.
wait_postgres_ready
wait_rustfs_ready

log "creating the RustFS bucket"
create_rustfs_bucket "http://$RUSTFS_C:9000"

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
