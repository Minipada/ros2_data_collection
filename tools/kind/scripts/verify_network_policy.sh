#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# The policy-enforced counterpart to deploy/robot/scripts/verify_network_isolation.sh
# (#452): that script proves the fleet's routing claims at the Podman-network level (no
# default route out — a missing route, not a firewall rule per connection); this proves
# the same claims where a CNI (Calico) actively denies a connection via NetworkPolicy.
# Requires kubernetes/{namespaces,networkpolicies,hub-postgres,edge-a,probes}.yaml
# already applied and ready — scripts/run.sh is what gets a cluster into that state.
set -euo pipefail

NS_ROBOT=dc-robot-a
NS_EDGE_A=dc-edge-a
NS_EDGE_B=dc-edge-b
NS_HUB=dc-hub
TIMEOUT="${DC_KIND_PROBE_TIMEOUT_SECONDS:-5}"

log() { echo "[verify-network-policy $(date -u +%H:%M:%S)] $*"; }

# probe_from <namespace> <pod> <host> <port>
probe_from() {
  kubectl exec -n "$1" "$2" -- timeout "$TIMEOUT" nc -z -w "$TIMEOUT" "$3" "$4" 2>/dev/null
}

expect_denied() {
  local desc="$1" ns="$2" pod="$3" host="$4" port="$5"
  if probe_from "$ns" "$pod" "$host" "$port"; then
    log "FAIL: $desc — connection succeeded, expected NetworkPolicy to deny it"
    exit 1
  fi
  log "PASS: $desc"
}

expect_allowed() {
  local desc="$1" ns="$2" pod="$3" host="$4" port="$5"
  if ! probe_from "$ns" "$pod" "$host" "$port"; then
    log "FAIL: $desc — connection failed, expected NetworkPolicy to allow it"
    exit 1
  fi
  log "PASS: $desc"
}

log "check 1: robot tier has no internet route"
expect_denied "robot -> public internet (1.1.1.1:443)" "$NS_ROBOT" robot-a-probe 1.1.1.1 443

log "check 2: site-to-site connectivity is denied, both directions"
expect_denied "robot-a -> edge-b (a different site)" "$NS_ROBOT" robot-a-probe "edge-b-probe.$NS_EDGE_B.svc.cluster.local" 8080
expect_denied "edge-b -> edge-a (a different site)" "$NS_EDGE_B" edge-b-probe "edge-vector.$NS_EDGE_A.svc.cluster.local" 24224
expect_denied "edge-b -> hub (edge-b is not an allow-listed site)" "$NS_EDGE_B" edge-b-probe "hub-postgres.$NS_HUB.svc.cluster.local" 5432

log "check 3: robot-to-edge and edge-to-hub connectivity is permitted (positive controls)"
expect_allowed "robot-a -> edge-a" "$NS_ROBOT" robot-a-probe "edge-vector.$NS_EDGE_A.svc.cluster.local" 24224
expect_allowed "edge-a -> hub" "$NS_EDGE_A" edge-a-probe "hub-postgres.$NS_HUB.svc.cluster.local" 5432

log "PASS: NetworkPolicy enforces every isolation and connectivity claim"
