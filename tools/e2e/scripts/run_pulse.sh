#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# One-command E2E smoke for the dc_measurements/TopicStatistics Measurement against a
# live ros2_pulse producer (github.com/TanayK07/ros2_pulse) — the real /statistics
# producer the plugin's unit tests only ever fake. From the repo root:
#
#   ./tools/e2e/scripts/run_pulse.sh
#
# Sibling of run.sh, not a mode inside it (own container, own gates). Runs ONE podman
# container that starts pulse_bridge, an LD_PRELOAD'd producer, and the DC stack
# (params/pulse_smoke_params.yaml) side by side, then hard-fails unless a Record carrying
# `average` and `sample_count` lands on /dc/measurement/topic_statistics. No Postgres,
# no RustFS, no loss accounting — the zero-loss harness (run.sh) owns those; this only
# proves the live-producer path the unit tests can't. Not wired into ci.yaml's heavy
# jobs: ci.yaml runs this same script as its own small smoke job (e2e-topic-statistics).
#
# ros2_pulse has no rolling apt package, so the container-side script, the params and
# the assertion subscriber are baked into the image (Containerfile.e2e source-builds the
# pinned SHA); this script only resolves the image and runs it. Podman only, no compose
# (CLAUDE.md "Containers: Podman, not Docker").
#
# Env vars (all optional):
#   DC_E2E_IMAGE                   a prebuilt dc-e2e image ref to run (CI sets this to
#                                  the :<sha> image its build-e2e-image job pushed, so
#                                  the smoke runs the exact built artifact). Pulled if
#                                  missing. Unset: dc-e2e:pulse, built here from
#                                  Containerfile.e2e on dc-workspace:latest if missing
#                                  (run tools/e2e/scripts/build.sh first if that base
#                                  is missing — building it here by surprise would
#                                  hide a 15-minute build behind a smoke).
#   DC_E2E_RECORD_TIMEOUT_SECONDS  how long to wait for the Record (default 120; the
#                                  producer's 5s window plus stack startup leaves room)
#   DC_E2E_TIMEOUT_SECONDS         hard cap on the whole container run (default 300)
#   DC_E2E_KEEP                    "true" to keep the exited container after a failure
#                                  (podman logs / podman cp) instead of removing it
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run"
mkdir -p "$RUN_DIR"

IMAGE="${DC_E2E_IMAGE:-dc-e2e:pulse}"
CONTAINER=dc_e2e_pulse
RECORD_TIMEOUT_SECONDS="${DC_E2E_RECORD_TIMEOUT_SECONDS:-120}"
TIMEOUT_SECONDS="${DC_E2E_TIMEOUT_SECONDS:-300}"
KEEP="${DC_E2E_KEEP:-false}"
FAILED=true

log() { echo "[$(date +%H:%M:%S)] $*"; }

cleanup() {
  if [ "$KEEP" = "true" ] && [ "$FAILED" = "true" ]; then
    log "DC_E2E_KEEP=true: keeping the exited $CONTAINER for debugging (podman logs $CONTAINER)"
    return
  fi
  # --ignore: equally quiet about a container the timeout cap already had SIGTERM'd.
  podman rm -f --ignore "$CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup EXIT

if [ -n "${DC_E2E_IMAGE:-}" ]; then
  podman image exists "$IMAGE" || podman pull "$IMAGE"
else
  if ! podman image exists "$IMAGE"; then
    podman image exists dc-workspace:latest ||
      {
        log "FAIL: neither $IMAGE nor dc-workspace:latest exists — build the workspace base first (tools/e2e/scripts/build.sh)"
        exit 1
      }
    log "building $IMAGE (Containerfile.e2e on dc-workspace:latest — source-builds ros2_pulse, so the first build is slow)"
    podman build --build-arg BASE_IMAGE=dc-workspace:latest -t "$IMAGE" \
      -f "$E2E_DIR/Containerfile.e2e" "$E2E_DIR"
  fi
fi

# --entrypoint, not a bare command: the image's ENTRYPOINT is the zero-loss harness's
# entrypoint.sh, which would swallow this script as extra dc_bringup launch arguments.
set +e
timeout "$TIMEOUT_SECONDS" podman run --network host --name "$CONTAINER" \
  --entrypoint /opt/e2e/pulse_smoke.sh \
  -e "DC_E2E_RECORD_TIMEOUT_SECONDS=$RECORD_TIMEOUT_SECONDS" \
  "$IMAGE" 2>&1 | tee "$RUN_DIR/pulse_smoke.log"
STATUS=$?
set -e

if [ "$STATUS" -ne 0 ]; then
  log "FAIL: TopicStatistics smoke exited $STATUS (124 means the ${TIMEOUT_SECONDS}s cap hit first) — full log: $RUN_DIR/pulse_smoke.log"
  exit 1
fi
FAILED=false
log "PASS: TopicStatistics E2E smoke"
