#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# One-command k3d dev loop (#451): a disposable single-node k3s cluster running the
# robot Pod from ../kubernetes/robot-pod.yaml. `podman kube play`
# (../scripts/verify_kube_play.sh, #450) runs that same manifest with no cluster at
# all; k3d adds the real thing it's missing — a scheduler, Services, DNS, `kubectl` —
# while still starting in seconds, which is what makes it right for iterating on the
# Kubernetes rendering itself. This is the fast loop, not the production-parity check:
# #452's kind cluster (Docker-based for the same reason, but minutes not seconds to
# bootstrap kubeadm) is what proves NetworkPolicy enforcement. See ../README.md.
#
# k3d wraps k3s in Docker containers, so this script depends on Docker — the one place
# this repository's containers touch Docker rather than Podman (CLAUDE.md "Containers:
# Podman, not Docker"). That dependency is scoped to this harness alone: building and
# shipping dc-ros/dc-uploader still goes through Podman (containers/dc-ros,
# containers/dc-uploader), unchanged, and nothing here runs in CI.
#
# Usage:
#   podman build -t dc-ros:dev -f containers/dc-ros/Containerfile containers/dc-ros
#   podman build -t dc-uploader:dev -f containers/dc-uploader/Containerfile containers/dc-uploader
#   ./deploy/robot/scripts/k3d_up.sh
#
# Re-running after rebuilding an image re-imports it and recreates the Pod against the
# same cluster, without tearing the cluster down — the fast half of "create and
# delete"; ../scripts/k3d_down.sh is the other half.
#
# Env vars:
#   DC_K3D_CLUSTER      cluster name (default dc-robot-dev)
#   DC_ROS_IMAGE        local dc-ros image ref to import (default dc-ros:dev)
#   DC_UPLOADER_IMAGE   local dc-uploader image ref to import (default dc-uploader:dev)
#   DC_K3D_READY_TIMEOUT_SECONDS  deadline for dc-ros to report ready (default 120 —
#                       first boot also pulls the upstream vector image over the
#                       network, unlike dc-ros/dc-uploader which are imported locally)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_DIR="$(dirname "$SCRIPT_DIR")"
POD_MANIFEST="$ROBOT_DIR/kubernetes/robot-pod.yaml"

CLUSTER="${DC_K3D_CLUSTER:-dc-robot-dev}"
DC_ROS_IMAGE="${DC_ROS_IMAGE:-dc-ros:dev}"
DC_UPLOADER_IMAGE="${DC_UPLOADER_IMAGE:-dc-uploader:dev}"
TIMEOUT_SECONDS="${DC_K3D_READY_TIMEOUT_SECONDS:-120}"
KUBE_CONTEXT="k3d-$CLUSTER"

log() { echo "[k3d-up $(date -u +%H:%M:%S)] $*"; }

if ! command -v k3d >/dev/null; then
  echo "k3d is required: https://k3d.io/#installation" >&2
  exit 1
fi

if k3d cluster list -o json | grep -q "\"name\":\"$CLUSTER\""; then
  log "cluster $CLUSTER already exists, reusing it"
else
  log "creating cluster $CLUSTER"
  # robot-pod.yaml's robot-params volume hostPath-mounts
  # /etc/dc/robot/robot_params.yaml (../scripts/verify_kube_play.sh's own hostPath
  # convention, so the manifest is identical between the two loops) — this maps that
  # path, inside the k3d server node, onto ../params on the real host. Set only at
  # cluster-create time, hence living here rather than in the manifest itself.
  #
  # KubeletInUserNamespace: without it, kubelet hard-fails on `open /dev/kmsg:
  # operation not permitted` on any host that doesn't expose /dev/kmsg to the node
  # container — rootless Docker, WSL2, devcontainers, some CI runners (verified
  # against this failure directly: cluster creation timed out on "cluster dns
  # configmap" until this flag was added). It's a node-level tolerance, not a
  # workload change, so it's on unconditionally rather than only under a flag —
  # harmless on a host that already has /dev/kmsg.
  k3d cluster create "$CLUSTER" \
    --servers 1 \
    --volume "$ROBOT_DIR/params:/etc/dc/robot@server:0" \
    --k3s-arg '--kubelet-arg=feature-gates=KubeletInUserNamespace=true@server:*' \
    --wait --timeout 60s
fi

# Images are built with Podman (CLAUDE.md's convention, unchanged); k3d's nodes are
# Docker containers, so `k3d image import` needs a tarball rather than a live Podman
# image reference. This save/import round trip is the "no registry" path the issue
# asks for — no push, no pull.
import_image() {
  local image="$1"
  if ! podman image exists "$image"; then
    log "skipping import: $image not found in local Podman storage (build it first)"
    return
  fi
  # Podman namespaces a locally built `name:tag` as `localhost/name:tag`; the Pod
  # manifest's bare `image: name:tag` instead normalizes to `docker.io/library/
  # name:tag` (the same default Docker itself applies) once it reaches containerd.
  # Without this retag the tarball lands under the wrong name and the kubelet reports
  # ErrImageNeverPull despite the import having "succeeded" (verified against that
  # exact failure).
  local full_ref="docker.io/library/$image"
  podman tag "$image" "$full_ref"
  local tar
  tar="$(mktemp -t k3d-image-XXXXXX.tar)"
  log "importing $image into $CLUSTER"
  podman save -o "$tar" "$full_ref"
  k3d image import "$tar" -c "$CLUSTER"
  rm -f "$tar"
}
import_image "$DC_ROS_IMAGE"
import_image "$DC_UPLOADER_IMAGE"

# Same image-ref substitution as ../scripts/verify_kube_play.sh, plus
# imagePullPolicy: Never on the two locally-imported images — the manifest's committed
# :latest tag defaults to Always, which would otherwise try (and fail — no registry is
# configured for this cluster) to pull on every apply instead of using the image k3d
# just imported.
RUN_MANIFEST="$(mktemp -t robot-pod.XXXXXX.yaml)"
trap 'rm -f "$RUN_MANIFEST"' EXIT
sed \
  -e "s|ghcr.io/minipada/ros2_data_collection/dc-ros:latest|$DC_ROS_IMAGE|" \
  -e "s|ghcr.io/minipada/ros2_data_collection/dc-uploader:latest|$DC_UPLOADER_IMAGE|" \
  "$POD_MANIFEST" > "$RUN_MANIFEST"
sed -i \
  -e "/image: $DC_ROS_IMAGE/a\\      imagePullPolicy: Never" \
  -e "/image: $DC_UPLOADER_IMAGE/a\\      imagePullPolicy: Never" \
  "$RUN_MANIFEST"

# Pod spec fields are immutable, so an in-place edit needs a delete first — cheap on a
# single-node k3d cluster and what makes re-running this script the fast iteration
# path: edit code, rebuild the image, rerun, without recreating the cluster.
if kubectl --context "$KUBE_CONTEXT" get pod dc-robot >/dev/null 2>&1; then
  log "replacing existing dc-robot Pod"
  kubectl --context "$KUBE_CONTEXT" delete pod dc-robot --wait --timeout=30s
fi

log "applying $RUN_MANIFEST"
kubectl --context "$KUBE_CONTEXT" apply -f "$RUN_MANIFEST"

# Same readiness signal as ../scripts/verify_kube_play.sh: dc-ros logging
# "dc_bridge reports ready", plus every container still running.
log "waiting up to ${TIMEOUT_SECONDS}s for dc-ros to report ready"
timeout "$TIMEOUT_SECONDS" bash -c "until kubectl --context $KUBE_CONTEXT logs dc-robot -c dc-ros 2>/dev/null | grep -q 'dc_bridge reports ready'; do sleep 1; done" \
  || { log "FAIL: dc-ros never reported ready"; kubectl --context "$KUBE_CONTEXT" logs dc-robot -c dc-ros --tail=50 || true; exit 1; }
log "PASS: dc-ros reports ready"

for c in dc-ros dc-uploader vector; do
  running="$(kubectl --context "$KUBE_CONTEXT" get pod dc-robot -o jsonpath="{.status.containerStatuses[?(@.name=='$c')].state.running}")"
  if [ -z "$running" ]; then
    log "FAIL: container $c is not running"
    kubectl --context "$KUBE_CONTEXT" logs dc-robot -c "$c" --tail=50 || true
    exit 1
  fi
done
log "PASS: dc-ros, dc-uploader and vector are all running in one Pod"
log "kubectl --context $KUBE_CONTEXT logs dc-robot -c dc-ros -f"
