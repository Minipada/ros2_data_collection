#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Limits harness: drain-rate axis (#385, part of #323's epic). From the repo root:
#
#   ./tools/e2e/scripts/run_limits_drain_rate.sh
#
# Answers "how long does a backlog take to clear once connectivity returns, as a
# function of load and outage length": brings up a standalone Shipper under test (a bare
# Vector instance, `fluent` source + `aws_s3` sink with a disk buffer, no dc_bridge/ROS
# involved — same convention run_limits_two_tier.sh's aggregating Shipper and
# run_load_driver_shipper_test.sh's bare Vector both use) plus RustFS as the Destination
# it can lose, then hands off to scripts/drain_rate_axis.py, which drives
# scripts/load_driver.py (#378) at a stated connection count/rate through an ascending
# ramp of outage lengths, inducing each outage with the exact `podman stop` / wait /
# `podman start` recipe run.sh itself uses on its own Postgres/RustFS containers (#385's
# "existing incident/outage scenario scaffolding, reused unmodified" acceptance
# criterion — this script calls podman directly with that same recipe rather than
# shelling out to run.sh, since run.sh's outage window is one step inside a much larger
# zero-loss run with its own fixed gates).
#
# For each outage length, drain_rate_axis.py:
#   1. stops RustFS, drives load for the outage's duration, sampling the Shipper's
#      buffer size throughout via its own `internal_metrics` + `prometheus_exporter`
#      (`vector_buffer_byte_size`) — not a filesystem stat on the buffer directory,
#      which was tried first and found not to work: Vector's on-disk buffer segment
#      file does not shrink as events are acked (see drain_rate_axis.py's module
#      docstring for the empirical finding). Injecting a metrics source/sink into this
#      standalone Vector config is not a DC code change — see #323's PRD note that
#      "observability comes from configuration, not code changes";
#   2. runs that sample window through saturation_probe.evaluate() (#377) with
#      `outage_declared=True` and hard-fails the whole run if it is ever reported
#      saturated — a growing buffer during a declared outage must never be misreported,
#      and this is the same module and the same disk-buffer signal #377 defines, not a
#      bespoke check (#385's other acceptance criterion);
#   3. restarts RustFS, polls the buffer size until it clears (or a wait bound expires),
#      and records that elapsed time as the drain time;
#   4. evaluates the *recovery* window with the identical probe/signal
#      (`outage_declared=False`) to decide whether this outage length is the ramp's knee
#      — the shortest outage length whose backlog failed to drain.
#
# Output feeds curve_reporter.py (#380) in the same stable AxisRun/CurveReport shape the
# other axes use; the per-level drain times themselves (curve_reporter has no field for
# them — see drain_rate_axis.py's module docstring) land in a companion JSON.
#
# Env vars (all optional):
#   DC_E2E_DRAIN_CONNECTIONS      synthetic load_driver.py connection count -- the
#                                 "stated load" (default 20)
#   DC_E2E_DRAIN_RATE_HZ          per-connection Records/s (default 10)
#   DC_E2E_DRAIN_OUTAGE_LENGTHS   comma-separated ascending outage lengths in seconds to
#                                 ramp through (default "30,60")
#   DC_E2E_DRAIN_POLL_INTERVAL_SECONDS   buffer-size sampling interval (default 5)
#   DC_E2E_DRAIN_MAX_WAIT_SECONDS        how long to wait for the backlog to clear after
#                                        RustFS comes back before giving up on this
#                                        outage length (default 120)
#   DC_E2E_DRAIN_SHIPPER_PORT / DC_E2E_DRAIN_METRICS_PORT   the Shipper-under-test's
#                                 host-published fluent/metrics ports (default 24236 /
#                                 9648 -- deliberately not the conventional 24224/9598,
#                                 to avoid colliding with another scenario script's own
#                                 Shipper on a shared host)
#   DC_E2E_KEEP                  "true" to leave the stack up after a failure for debugging
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(cd "$E2E_DIR/../.." && pwd)"
RUN_DIR="$E2E_DIR/.run/limits_drain_rate"

CONNECTIONS="${DC_E2E_DRAIN_CONNECTIONS:-20}"
RATE_HZ="${DC_E2E_DRAIN_RATE_HZ:-10}"
OUTAGE_LENGTHS="${DC_E2E_DRAIN_OUTAGE_LENGTHS:-30,60}"
POLL_INTERVAL_SECONDS="${DC_E2E_DRAIN_POLL_INTERVAL_SECONDS:-5}"
MAX_WAIT_SECONDS="${DC_E2E_DRAIN_MAX_WAIT_SECONDS:-120}"
KEEP="${DC_E2E_KEEP:-false}"

# A dedicated bridge network, not --network host (unlike run.sh et al.): this scenario's
# RustFS never needs to be host-reachable at all (only the Shipper talks to it, over
# container-name DNS), so keeping it off the host namespace entirely avoids contending
# with any other scenario script's own RustFS for host port 9000 — the same reasoning
# run_limits_two_tier.sh's own bridge network is built on. Only the Shipper's fluent and
# metrics ports are published to the host, for this script's own load_driver.py/curl
# calls.
NET=dc-e2e-drain-net
RUSTFS_C=dc-e2e-drain-rustfs
SHIPPER_C=dc-e2e-drain-shipper
# Not the conventional 24224/9598: this Shipper under test is a self-contained loop
# (only this script's own load_driver.py invocation ever connects to it), so a
# less-contended port pair avoids colliding with any other scenario script's own
# Shipper/exporter published on this host, even though RustFS itself no longer needs a
# host port at all (see $NET above).
SHIPPER_PORT="${DC_E2E_DRAIN_SHIPPER_PORT:-24236}"
METRICS_PORT="${DC_E2E_DRAIN_METRICS_PORT:-9648}"
BUFFER_ID=to_object_storage
BUCKET=dc-e2e

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

log() { echo "[e2e-drain-rate $(date -u +%H:%M:%S)] $*"; }

remove_stack() {
  podman rm -f --ignore "$SHIPPER_C" "$RUSTFS_C" >/dev/null
  if podman volume exists dc_e2e_drain_rustfs_data; then
    podman volume rm dc_e2e_drain_rustfs_data >/dev/null
  fi
  podman network rm "$NET" >/dev/null 2>&1 || true
}

cleanup() {
  local exit_code=$?
  if [ "$exit_code" -ne 0 ] && [ "$KEEP" = "true" ]; then
    log "FAILED (exit $exit_code) — leaving the stack up (DC_E2E_KEEP=true) for debugging"
    exit "$exit_code"
  fi
  log "tearing down"
  if podman container exists "$SHIPPER_C"; then
    podman logs "$SHIPPER_C" > "$RUN_DIR/shipper.log" 2>&1 || true
  fi
  remove_stack
  exit "$exit_code"
}
trap cleanup EXIT

# The exact Vector version dc_bridge is built and tested against, same as
# run_limits_two_tier.sh / run_load_driver_shipper_test.sh -- read out of the .repos pin
# that fetches vector_vendor, not duplicated as a second source of truth.
VECTOR_VERSION="$(grep -A3 'vector_vendor:' "$REPO_ROOT/ros2_data_collection.repos" | grep -oP 'version: v\K[0-9.]+')"
VECTOR_IMAGE="docker.io/timberio/vector:${VECTOR_VERSION}-debian"

remove_stack

log "creating the bridge network ($NET)"
podman network create "$NET" >/dev/null

# --- RustFS: the Destination this scenario induces an outage on, internal-only --------
log "starting RustFS on $NET (no host port published)"
podman run -d --network "$NET" --name "$RUSTFS_C" \
  -v dc_e2e_drain_rustfs_data:/data \
  docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c >/dev/null

log "waiting for RustFS to accept TCP connections on $NET"
timeout 60 bash -c "
  until podman run --rm --network $NET --entrypoint bash '$VECTOR_IMAGE' -c 'exec 3<>/dev/tcp/$RUSTFS_C/9000' >/dev/null 2>&1; do
    sleep 1
  done
" || { log "FAIL: RustFS never became reachable on $NET"; exit 1; }

log "creating the RustFS bucket"
podman run --rm --network "$NET" \
  -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
  docker.io/amazon/aws-cli:latest \
  --endpoint-url "http://$RUSTFS_C:9000" s3 mb "s3://$BUCKET"

# --- Shipper under test: bare Vector, fluent source + aws_s3 sink with a disk buffer ---
log "rendering the Shipper-under-test's Vector config"
cat > "$RUN_DIR/shipper_vector.toml" <<EOF
data_dir = "/var/lib/vector"

[acknowledgements]
enabled = true

[sources.load]
type = "fluent"
address = "0.0.0.0:${SHIPPER_PORT}"

[sources.internal_metrics]
type = "internal_metrics"

[sinks.${BUFFER_ID}]
type = "aws_s3"
inputs = ["load"]
bucket = "${BUCKET}"
endpoint = "http://${RUSTFS_C}:9000"
region = "us-east-1"
force_path_style = true
key_prefix = "drain-rate/"

[sinks.${BUFFER_ID}.auth]
access_key_id = "rustfsadmin"
secret_access_key = "rustfsadmin"

[sinks.${BUFFER_ID}.batch]
timeout_secs = 2

[sinks.${BUFFER_ID}.buffer]
type = "disk"
max_size = 268435488

[sinks.${BUFFER_ID}.encoding]
codec = "json"

[sinks.expose_metrics]
type = "prometheus_exporter"
inputs = ["internal_metrics"]
address = "0.0.0.0:${METRICS_PORT}"
EOF

log "starting the Shipper under test ($SHIPPER_C, Vector $VECTOR_VERSION)"
podman run -d --network "$NET" -p "${SHIPPER_PORT}:${SHIPPER_PORT}" -p "${METRICS_PORT}:${METRICS_PORT}" \
  --name "$SHIPPER_C" \
  -v "$RUN_DIR/shipper_vector.toml:/etc/vector/vector.toml:Z" \
  "$VECTOR_IMAGE" -c /etc/vector/vector.toml >/dev/null

log "waiting for the Shipper's fluent source to accept connections"
if ! timeout 30 bash -c "until (exec 3<>/dev/tcp/127.0.0.1/${SHIPPER_PORT}) 2>/dev/null; do sleep 0.5; done"; then
  log "FAIL: the Shipper under test never started listening on port $SHIPPER_PORT"
  exit 1
fi

log "waiting for the Shipper's metrics endpoint to respond"
if ! timeout 30 bash -c "until curl -sf http://127.0.0.1:${METRICS_PORT}/metrics >/dev/null 2>&1; do sleep 0.5; done"; then
  log "FAIL: the Shipper under test's prometheus_exporter never came up on port $METRICS_PORT"
  exit 1
fi

# --- drive the ramp -----------------------------------------------------------------
log "running the drain-rate axis: $CONNECTIONS synthetic connection(s) @ ${RATE_HZ}Hz, outage lengths [$OUTAGE_LENGTHS]s"
if ! (cd "$REPO_ROOT" && uv run --frozen python3 tools/e2e/scripts/drain_rate_axis.py \
  --rustfs-container "$RUSTFS_C" \
  --shipper-host 127.0.0.1 \
  --shipper-port "$SHIPPER_PORT" \
  --metrics-url "http://127.0.0.1:${METRICS_PORT}/metrics" \
  --buffer-id "$BUFFER_ID" \
  --connections "$CONNECTIONS" \
  --rate "$RATE_HZ" \
  --outage-lengths "$OUTAGE_LENGTHS" \
  --poll-interval "$POLL_INTERVAL_SECONDS" \
  --max-drain-wait "$MAX_WAIT_SECONDS" \
  --repo-root "$REPO_ROOT" \
  --report-json "$RUN_DIR/curve_report.json" \
  --summary-txt "$RUN_DIR/curve_summary.txt" \
  --measurements-json "$RUN_DIR/drain_measurements.json"); then
  log "FAIL: the drain-rate axis run failed — see output above"
  exit 1
fi

log "PASS: drain-rate axis (#385) — report: $RUN_DIR/curve_report.json, measurements: $RUN_DIR/drain_measurements.json"
cat "$RUN_DIR/curve_summary.txt"
