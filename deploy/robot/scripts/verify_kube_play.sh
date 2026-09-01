#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Runtime-free check for the Kubernetes rendering (#450): `podman kube play` runs
# ../kubernetes/robot-pod.yaml directly — no cluster, possible only because the robot
# tier is one Pod (see that file's header) — and this asserts it reaches ready the
# same way tools/release/scripts/verify_published_images.sh does: dc-ros logging
# "dc_bridge reports ready", plus every container still running. Cluster-backed
# validation (a real scheduler, Services, DNS, NetworkPolicy) is #451/#452, not this
# script.
#
# Env vars:
#   DC_ROS_IMAGE       image ref for the dc-ros container (default: the manifest's own
#                      committed :latest ref). CI passes the :<sha> ref
#                      build-dc-ros-image just pushed, so a PR run tests the image it
#                      just built, not whatever :latest happened to be last.
#   DC_UPLOADER_IMAGE  same, for the dc-uploader container.
#   DC_RELEASE_TIMEOUT_SECONDS  deadline for dc-ros to report ready (default 60).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_DIR="$(dirname "$SCRIPT_DIR")"
POD_MANIFEST="$ROBOT_DIR/kubernetes/robot-pod.yaml"
TIMEOUT_SECONDS="${DC_RELEASE_TIMEOUT_SECONDS:-60}"

POD_NAME=dc-robot
DC_ROS_C="${POD_NAME}-dc-ros"
UPLOADER_C="${POD_NAME}-dc-uploader"
VECTOR_C="${POD_NAME}-vector"

# robot-pod.yaml's dc-ros container hostPath-mounts robot_params.yaml from a fixed
# host path (matches quadlet/dc-ros.container's own convention) — `podman kube play`
# has no context-directory substitution, so this script has to put the file there
# itself rather than the manifest referencing a relative path.
HOST_PARAMS_DIR=/etc/dc/robot
HOST_PARAMS_FILE="$HOST_PARAMS_DIR/robot_params.yaml"

RUN_MANIFEST="$(mktemp -t robot-pod.XXXXXX.yaml)"

log() { echo "[verify-kube-play $(date -u +%H:%M:%S)] $*"; }

cleanup() {
  local exit_code=$?
  log "tearing down"
  # Pod name is fixed regardless of which image refs were substituted in, so teardown
  # doesn't need the (possibly already-removed) temp manifest.
  podman pod rm -f "$POD_NAME" >/dev/null 2>&1 || true
  rm -f "$RUN_MANIFEST"
  exit "$exit_code"
}
trap cleanup EXIT

# Substitute image refs only when overridden — an unset DC_ROS_IMAGE/DC_UPLOADER_IMAGE
# leaves the manifest's own committed :latest refs untouched, so a plain local run
# behaves exactly as the committed file describes.
cp "$POD_MANIFEST" "$RUN_MANIFEST"
if [ -n "${DC_ROS_IMAGE:-}" ]; then
  sed -i "s|ghcr.io/minipada/ros2_data_collection/dc-ros:latest|$DC_ROS_IMAGE|" "$RUN_MANIFEST"
fi
if [ -n "${DC_UPLOADER_IMAGE:-}" ]; then
  sed -i "s|ghcr.io/minipada/ros2_data_collection/dc-uploader:latest|$DC_UPLOADER_IMAGE|" "$RUN_MANIFEST"
fi

log "staging robot_params.yaml at $HOST_PARAMS_FILE"
sudo install -d -m 0755 "$HOST_PARAMS_DIR"
sudo install -m 0644 "$ROBOT_DIR/params/robot_params.yaml" "$HOST_PARAMS_FILE"

log "podman kube play $RUN_MANIFEST"
podman kube play "$RUN_MANIFEST"

log "waiting up to ${TIMEOUT_SECONDS}s for dc-ros to report ready"
timeout "$TIMEOUT_SECONDS" bash -c "until podman logs $DC_ROS_C 2>&1 | grep -q 'dc_bridge reports ready'; do sleep 1; done" \
  || { log "FAIL: dc-ros never reported ready"; podman logs "$DC_ROS_C" 2>&1 | tail -n 50; exit 1; }
log "PASS: dc-ros reports ready"

for c in "$DC_ROS_C" "$UPLOADER_C" "$VECTOR_C"; do
  if [ "$(podman inspect --format '{{.State.Running}}' "$c")" != "true" ]; then
    log "FAIL: $c is not running"
    podman logs "$c" 2>&1 | tail -n 50
    exit 1
  fi
done
log "PASS: dc-ros, dc-uploader and vector are all running under one Pod"
