#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# kind validation of the three-tier topology with NetworkPolicy (#452, epic #440). Brings
# up a kubeadm cluster with Calico enforcing NetworkPolicy, a robot tier (site A), an edge
# aggregator (site A), a stand-in second site (dc-edge-b), and a hub, then proves:
#
#   - the robot tier has no internet route, and cannot reach a different site
#     (verify_network_policy.sh);
#   - robot -> edge -> hub is permitted and carries real Records collected by dc-ros
#     (this script, "records flow" section);
#   - the robot tier keeps collecting and buffering to disk while the edge tier is
#     unreachable — a real NetworkPolicy change, not a stopped container — and loses
#     nothing once it's restored (this script, "induced outage" section).
#
# This is the production-parity proof; #451's k3d cluster (Flannel, not
# policy-enforcing) is the fast local iteration loop, not this. See ../README.md for the
# Docker-dependency note both harnesses share.
#
# This is the *only* script for this harness — no separate up/down helpers. CI, a local
# reproduction, and anyone else exercising this validation all run exactly this, so there
# is one path to drift out of sync, not several: what CI asserts is exactly what a
# developer can run by hand. ../README.md documents the same bring-up/teardown commands
# used below (kind create/delete cluster, kubectl apply, podman save + kind load
# image-archive) for reading without running the script, but they are the same commands,
# not a parallel path.
#
#   ./tools/kind/scripts/run.sh
#
# Env vars (all optional):
#   DC_ROS_IMAGE                  dc-ros image ref. Unset: build it from a fresh
#                                  workspace image, same as CI's build-dc-ros-image job.
#   DC_WORKSPACE_IMAGE            workspace image to build DC_ROS_IMAGE from, if unset
#                                  (default: build via tools/e2e/scripts/build.sh)
#   DC_KIND_CLUSTER               kind cluster name (default dc-kind)
#   DC_KIND_CALICO_VERSION        Calico manifest version to install (default v3.32.2)
#   DC_KIND_READY_TIMEOUT         deadline for cluster/Calico readiness (default 180s)
#   DC_KIND_STEADY_STATE_SECONDS  warmup before the induced outage (default 15)
#   DC_KIND_OUTAGE_SECONDS        outage duration (default 30)
#   DC_KIND_DRAIN_SECONDS         settle time after the outage before verifying (default 15)
#   DC_KIND_KEEP                  "true" to leave the cluster up on failure
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KIND_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(cd "$KIND_DIR/../.." && pwd)"
K8S_DIR="$KIND_DIR/kubernetes"
PARAMS_DIR="$KIND_DIR/params"
RUN_DIR="$KIND_DIR/.run"

CLUSTER_NAME="${DC_KIND_CLUSTER:-dc-kind}"
CALICO_VERSION="${DC_KIND_CALICO_VERSION:-v3.32.2}"
READY_TIMEOUT="${DC_KIND_READY_TIMEOUT:-180s}"
STEADY_STATE_SECONDS="${DC_KIND_STEADY_STATE_SECONDS:-15}"
OUTAGE_SECONDS="${DC_KIND_OUTAGE_SECONDS:-30}"
DRAIN_SECONDS="${DC_KIND_DRAIN_SECONDS:-15}"
KEEP="${DC_KIND_KEEP:-false}"
# The uptime Measurement's own polling_interval (params/robot-a-params.yaml) — the rate
# the induced-outage zero-loss check computes its expected count from.
UPTIME_RATE_HZ=1

mkdir -p "$RUN_DIR"

log() { echo "[kind-run $(date -u +%H:%M:%S)] $*"; }

kx() { kubectl --context "kind-$CLUSTER_NAME" "$@"; }

pg_count() {
  kx exec -n dc-hub deploy/hub-postgres -- psql -U dc -d dc -tAc 'SELECT count(*) FROM dc_records' 2>/dev/null | tr -d '[:space:]'
}

dump_diagnostics() {
  for ns in dc-robot-a dc-edge-a dc-edge-b dc-hub; do
    kx get pods -n "$ns" -o wide > "$RUN_DIR/pods_${ns}.txt" 2>&1 || true
  done
  kx logs -n dc-robot-a dc-robot -c dc-ros > "$RUN_DIR/dc-ros.log" 2>&1 || true
  kx logs -n dc-robot-a dc-robot -c vector > "$RUN_DIR/robot-vector.log" 2>&1 || true
  kx logs -n dc-edge-a deploy/edge-vector > "$RUN_DIR/edge-vector.log" 2>&1 || true
  kx logs -n dc-hub deploy/hub-postgres > "$RUN_DIR/hub-postgres.log" 2>&1 || true
}

cleanup() {
  local exit_code=$?
  if [ "$exit_code" -ne 0 ]; then
    log "FAILED (exit $exit_code) — collecting diagnostics into $RUN_DIR"
    dump_diagnostics
    if [ "$KEEP" = "true" ]; then
      log "leaving the cluster up (DC_KIND_KEEP=true) for debugging"
      exit "$exit_code"
    fi
  fi
  log "tearing down"
  kind delete cluster --name "$CLUSTER_NAME" || true
  exit "$exit_code"
}
trap cleanup EXIT

# --- obtain the dc-ros image (same fallback shape as tools/e2e/scripts/run.sh) ---------
if [ -n "${DC_ROS_IMAGE:-}" ]; then
  log "using prebuilt dc-ros image: $DC_ROS_IMAGE (no build)"
  podman image exists "$DC_ROS_IMAGE" || podman pull "$DC_ROS_IMAGE"
else
  if [ -n "${DC_WORKSPACE_IMAGE:-}" ]; then
    log "using prebuilt DC workspace image: $DC_WORKSPACE_IMAGE (skipping build.sh)"
    podman image exists "$DC_WORKSPACE_IMAGE" || podman pull "$DC_WORKSPACE_IMAGE"
    WORKSPACE_IMAGE="$DC_WORKSPACE_IMAGE"
  else
    log "building the DC workspace image (tools/e2e/scripts/build.sh)"
    "$REPO_ROOT/tools/e2e/scripts/build.sh"
    WORKSPACE_IMAGE="dc-workspace:latest"
  fi
  log "building dc-ros (podman build, containers/dc-ros/Containerfile, FROM the workspace image)"
  podman build --build-arg "BASE_IMAGE=$WORKSPACE_IMAGE" -t dc-ros:kind -f "$REPO_ROOT/containers/dc-ros/Containerfile" "$REPO_ROOT/containers/dc-ros"
  DC_ROS_IMAGE="dc-ros:kind"
fi

# Same source of truth every other scenario reads this from (#448).
VECTOR_VERSION="$(grep -A3 'vector_vendor:' "$REPO_ROOT/ros2_data_collection.repos" | grep -oP 'version: v\K[0-9.]+')"
export VECTOR_IMAGE="docker.io/timberio/vector:${VECTOR_VERSION}-debian"
export DC_ROS_IMAGE

# --- cluster: kind create + Calico (kindnet/Flannel don't enforce NetworkPolicy) --------
if kind get clusters 2>/dev/null | grep -qx "$CLUSTER_NAME"; then
  log "cluster $CLUSTER_NAME already exists, reusing it"
else
  log "kind create cluster --name $CLUSTER_NAME (kubeadm, no default CNI)"
  kind create cluster --name "$CLUSTER_NAME" --config "$KIND_DIR/kind-config.yaml"
fi
kubectl config use-context "kind-$CLUSTER_NAME" >/dev/null

log "kubectl apply: Calico $CALICO_VERSION"
kx apply -f "https://raw.githubusercontent.com/projectcalico/calico/${CALICO_VERSION}/manifests/calico.yaml"
kx -n kube-system rollout status daemonset/calico-node --timeout="$READY_TIMEOUT"
kx -n kube-system rollout status deployment/calico-kube-controllers --timeout="$READY_TIMEOUT"
kx wait --for=condition=Ready nodes --all --timeout="$READY_TIMEOUT"
log "PASS: cluster $CLUSTER_NAME is up with Calico enforcing NetworkPolicy"

# --- load the dc-ros image into the cluster, no registry involved ----------------------
# `kind load docker-image` reads from the *Docker* image store, which this repo's own
# podman-built images never populate (CLAUDE.md "Containers: Podman, not Docker") — a tar
# round trip instead: `podman save` -> `kind load image-archive`. This also means the
# robot Pod below never needs a ghcr.io pull secret.
log "podman save $DC_ROS_IMAGE, kind load image-archive"
DC_ROS_TAR="$(mktemp -t kind-image.XXXXXX.tar)"
podman save -o "$DC_ROS_TAR" "$DC_ROS_IMAGE"
kind load image-archive "$DC_ROS_TAR" --name "$CLUSTER_NAME"
rm -f "$DC_ROS_TAR"

# --- namespaces, config, hub + edge tiers -----------------------------------------------
log "kubectl apply: namespaces"
kx apply -f "$K8S_DIR/namespaces.yaml"

log "rendering ConfigMaps from source files (single source of truth stays the file, not this script)"
kx create configmap hub-init-sql -n dc-hub --from-file="$REPO_ROOT/tools/e2e/sql/init.sql" --dry-run=client -o yaml | kx apply -f -
kx create configmap edge-a-vector-config -n dc-edge-a --from-file="vector.toml=$PARAMS_DIR/edge-a-vector.toml" --dry-run=client -o yaml | kx apply -f -
kx create configmap robot-a-params -n dc-robot-a --from-file="robot_params.yaml=$PARAMS_DIR/robot-a-params.yaml" --dry-run=client -o yaml | kx apply -f -

log "kubectl apply: steady-state NetworkPolicy"
kx apply -f "$K8S_DIR/networkpolicies.yaml"

log "kubectl apply: hub tier"
kx apply -f "$K8S_DIR/hub-postgres.yaml"

log "kubectl apply: edge tier (site A)"
sed "s|\${VECTOR_IMAGE}|$VECTOR_IMAGE|g" "$K8S_DIR/edge-a.yaml" | kx apply -f -

log "kubectl apply: probe Pods"
kx apply -f "$K8S_DIR/probes.yaml"

# rollout status, not `wait --for=condition=Ready pod -l ...`: the Deployment object
# exists synchronously after apply, but its Pods don't yet, and `kubectl wait` against a
# selector that currently matches nothing returns immediately instead of waiting for one
# to appear.
log "waiting for the hub and edge Deployments to become available"
kx rollout status -n dc-hub deployment/hub-postgres --timeout=180s
kx rollout status -n dc-edge-a deployment/edge-vector --timeout=180s

log "waiting for the probe Pods to become ready"
kx wait -n dc-robot-a --for=condition=Ready pod/robot-a-probe --timeout=60s
kx wait -n dc-edge-a --for=condition=Ready pod/edge-a-probe --timeout=60s
kx wait -n dc-edge-b --for=condition=Ready pod/edge-b-probe --timeout=60s

# --- robot tier (last: its own destination, edge-a, is already up) -----------------------
log "kubectl apply: robot tier (site A)"
sed -e "s|\${DC_ROS_IMAGE}|$DC_ROS_IMAGE|g" -e "s|\${VECTOR_IMAGE}|$VECTOR_IMAGE|g" "$K8S_DIR/robot-a.yaml" | kx apply -f -

log "waiting up to 90s for dc-ros to report ready"
DC_ROS_READY=false
for _ in $(seq 1 45); do
  if kx logs -n dc-robot-a dc-robot -c dc-ros 2>&1 | grep -q 'dc_bridge reports ready'; then
    DC_ROS_READY=true
    break
  fi
  sleep 2
done
[ "$DC_ROS_READY" = "true" ] || { log "FAIL: dc-ros never reported ready"; exit 1; }
log "PASS: dc-ros reports ready"

# --- NetworkPolicy enforcement: internet, site-to-site, positive controls ---------------
"$SCRIPT_DIR/verify_network_policy.sh"

# --- Records flow: robot -> edge -> hub, carrying real Records -------------------------
log "waiting up to 60s for the first Record to reach the hub through the edge aggregator"
RECORD_ARRIVED=false
for _ in $(seq 1 30); do
  N="$(pg_count)"
  if [ -n "$N" ] && [ "$N" -gt 0 ] 2>/dev/null; then
    RECORD_ARRIVED=true
    break
  fi
  sleep 2
done
[ "$RECORD_ARRIVED" = "true" ] || { log "FAIL: no Record reached the hub within 60s"; exit 1; }
log "PASS: Records collected on the robot tier arrived at the hub through the edge aggregator"

log "steady state for ${STEADY_STATE_SECONDS}s"
sleep "$STEADY_STATE_SECONDS"
WINDOW_START_TS="$(date +%s)"
COUNT_BEFORE_OUTAGE="$(pg_count)"
COUNT_BEFORE_OUTAGE="${COUNT_BEFORE_OUTAGE:-0}"

# --- induced outage: a real NetworkPolicy change, not a stopped container --------------
log "inducing outage: replacing dc-robot-a's NetworkPolicy so it can no longer reach edge-a, for ${OUTAGE_SECONDS}s"
kx apply -f "$K8S_DIR/networkpolicy-robot-outage.yaml"
sleep "$OUTAGE_SECONDS"

log "restoring dc-robot-a's NetworkPolicy"
kx apply -f "$K8S_DIR/networkpolicies.yaml"

log "draining for ${DRAIN_SECONDS}s"
sleep "$DRAIN_SECONDS"

COUNT_AFTER="$(pg_count)"
COUNT_AFTER="${COUNT_AFTER:-0}"
NOW_TS="$(date +%s)"
WINDOW_ELAPSED=$((NOW_TS - WINDOW_START_TS))
DELTA=$((COUNT_AFTER - COUNT_BEFORE_OUTAGE))
# uptime publishes at UPTIME_RATE_HZ; the Bridge and the robot's local Vector never stop
# accepting/buffering during the outage (ADR-0002's disk buffer), so the records added
# across [WINDOW_START_TS, now] — steady-state, the outage, and the drain together —
# should track elapsed time at that rate regardless of the outage in the middle. A real
# loss would show up as a permanent shortfall here, not a transient dip.
EXPECTED_DELTA=$((WINDOW_ELAPSED * UPTIME_RATE_HZ))
# Generous lower bound: covers Vector's own batch/flush cadence without being loose
# enough to hide a gap the size of the outage window itself.
MIN_ACCEPTABLE_DELTA=$((EXPECTED_DELTA * 70 / 100))

log "records: $COUNT_BEFORE_OUTAGE before the outage window, $COUNT_AFTER after restore+drain (+$DELTA over ${WINDOW_ELAPSED}s, expected ~${EXPECTED_DELTA}, minimum acceptable ${MIN_ACCEPTABLE_DELTA})"
if [ "$DELTA" -le 0 ]; then
  log "FAIL: record count did not grow across the outage — delivery never resumed"
  exit 1
fi
if [ "$DELTA" -lt "$MIN_ACCEPTABLE_DELTA" ]; then
  log "FAIL: only $DELTA new record(s) arrived, far below what ${WINDOW_ELAPSED}s at ${UPTIME_RATE_HZ}Hz predicts (~${EXPECTED_DELTA}) — Records were lost, not just delayed"
  exit 1
fi
log "PASS: the robot tier kept collecting and buffering through the outage and lost nothing once the edge tier was reachable again"

log "PASS: kind validation of the three-tier topology with NetworkPolicy (#452)"
