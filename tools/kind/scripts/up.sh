#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Creates the #452 kind cluster and installs Calico as its CNI. kind is Docker-based
# (this repo's own building and shipping stays on Podman, unchanged — see
# ../README.md's "Docker dependency" section) and needs a real, policy-enforcing CNI:
# kind's own default (kindnet) and k3d's default (Flannel, #451) both silently accept
# NetworkPolicy objects without enforcing them, which would make every check in
# verify_network_policy.sh pass for the wrong reason. disableDefaultCNI in
# ../kind-config.yaml is what leaves the gap Calico fills here.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KIND_DIR="$(dirname "$SCRIPT_DIR")"

CLUSTER_NAME="${DC_KIND_CLUSTER:-dc-kind}"
CALICO_VERSION="${DC_KIND_CALICO_VERSION:-v3.32.2}"
TIMEOUT="${DC_KIND_READY_TIMEOUT:-180s}"

log() { echo "[kind-up $(date -u +%H:%M:%S)] $*"; }

if kind get clusters 2>/dev/null | grep -qx "$CLUSTER_NAME"; then
  log "cluster $CLUSTER_NAME already exists, reusing it"
else
  log "creating cluster $CLUSTER_NAME (kubeadm, no default CNI)"
  kind create cluster --name "$CLUSTER_NAME" --config "$KIND_DIR/kind-config.yaml"
fi

kubectl config use-context "kind-$CLUSTER_NAME" >/dev/null

log "installing Calico $CALICO_VERSION"
kubectl apply -f "https://raw.githubusercontent.com/projectcalico/calico/${CALICO_VERSION}/manifests/calico.yaml"

log "waiting up to $TIMEOUT for Calico to roll out"
kubectl -n kube-system rollout status daemonset/calico-node --timeout="$TIMEOUT"
kubectl -n kube-system rollout status deployment/calico-kube-controllers --timeout="$TIMEOUT"

log "waiting up to $TIMEOUT for nodes to report Ready (only true once the CNI is up)"
kubectl wait --for=condition=Ready nodes --all --timeout="$TIMEOUT"

log "PASS: cluster $CLUSTER_NAME is up with Calico enforcing NetworkPolicy"
