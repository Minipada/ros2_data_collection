#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Runtime-free proof of the fleet's routing claim (#450, epic #440 scenario 3): the
# robot tier has no internet route, and the only permitted connectivity is robot →
# edge, outbound. Plain Podman networks, no cluster, no CNI — this is a routing-level
# proof (an `--internal` network has no default route out, not a firewall rule per
# connection); the policy-enforced version, where a CNI actively denies rather than
# just not routing, is #452's kind + NetworkPolicy validation.
#
# Topology this script builds:
#   dc_robot_isolation_net (--internal, no route out) — robot_probe, edge_stub
#   dc_edge_isolation_net  (normal bridge, has a route out) — edge_stub, outsider
# edge_stub is dual-homed, standing in for the edge aggregator this repo does not ship
# (epic #440's "Out of Scope") — the one node the robot tier is allowed to reach.
set -euo pipefail

TIMEOUT_SECONDS="${DC_RELEASE_TIMEOUT_SECONDS:-15}"
IMAGE="docker.io/library/alpine:3.20"

ROBOT_NET=dc_robot_isolation_net
EDGE_NET=dc_edge_isolation_net
ROBOT_PROBE=dc_robot_isolation_probe
EDGE_STUB=dc_robot_isolation_edge_stub
OUTSIDER=dc_robot_isolation_outsider

log() { echo "[verify-network-isolation $(date -u +%H:%M:%S)] $*"; }

cleanup() {
  local exit_code=$?
  log "tearing down"
  podman rm -f "$ROBOT_PROBE" "$EDGE_STUB" "$OUTSIDER" >/dev/null 2>&1 || true
  podman network rm "$ROBOT_NET" "$EDGE_NET" >/dev/null 2>&1 || true
  exit "$exit_code"
}
trap cleanup EXIT

log "creating networks: $ROBOT_NET (--internal), $EDGE_NET (normal)"
podman network create --internal "$ROBOT_NET" >/dev/null
podman network create "$EDGE_NET" >/dev/null

log "starting edge_stub (dual-homed: robot-facing + edge-facing) and a TCP listener on it"
podman run -d --name "$EDGE_STUB" --network "$ROBOT_NET" "$IMAGE" sleep 300 >/dev/null
podman network connect "$EDGE_NET" "$EDGE_STUB"
# busybox nc has no reliable `-k`/keep-listening flag across builds; loop instead.
podman exec -d "$EDGE_STUB" sh -c 'while true; do nc -l -p 24224; done'

log "starting robot_probe, attached only to $ROBOT_NET"
podman run -d --name "$ROBOT_PROBE" --network "$ROBOT_NET" "$IMAGE" sleep 300 >/dev/null

log "starting outsider, attached only to $EDGE_NET (not $ROBOT_NET)"
podman run -d --name "$OUTSIDER" --network "$EDGE_NET" "$IMAGE" sleep 300 >/dev/null

EDGE_STUB_ROBOT_IP="$(podman inspect --format "{{.NetworkSettings.Networks.$ROBOT_NET.IPAddress}}" "$EDGE_STUB")"
ROBOT_PROBE_IP="$(podman inspect --format "{{.NetworkSettings.Networks.$ROBOT_NET.IPAddress}}" "$ROBOT_PROBE")"

log "check 1: robot_probe has no internet route"
if podman exec "$ROBOT_PROBE" timeout "$TIMEOUT_SECONDS" nc -z -w "$TIMEOUT_SECONDS" 1.1.1.1 443 2>/dev/null; then
  log "FAIL: robot_probe reached the public internet — --internal network is not isolating it"
  exit 1
fi
log "PASS: robot_probe cannot reach the public internet"

log "check 2: robot_probe can reach edge_stub (the one permitted outbound hop)"
if ! podman exec "$ROBOT_PROBE" nc -z -w "$TIMEOUT_SECONDS" "$EDGE_STUB_ROBOT_IP" 24224; then
  log "FAIL: robot_probe could not reach edge_stub over $ROBOT_NET"
  exit 1
fi
log "PASS: robot_probe reached edge_stub"

log "check 3: outsider (not on $ROBOT_NET) cannot reach robot_probe"
if podman exec "$OUTSIDER" timeout "$TIMEOUT_SECONDS" nc -z -w "$TIMEOUT_SECONDS" "$ROBOT_PROBE_IP" 1 2>/dev/null; then
  log "FAIL: outsider reached robot_probe — the robot network is reachable from outside its tier"
  exit 1
fi
log "PASS: outsider cannot reach robot_probe"

log "PASS: robot tier has no internet route, and only robot-to-edge connectivity is permitted"
