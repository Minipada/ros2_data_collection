#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Tears down the k3d dev-loop cluster ./k3d_up.sh created (#451). Deleting a k3d
# cluster removes its Docker containers, network and volumes in one shot — no
# manifest to walk back the way `kubectl delete` would need — which is what makes the
# create/delete cycle fast enough to repeat instead of leaving the cluster running
# between sessions.
#
# Env vars:
#   DC_K3D_CLUSTER  cluster name (default dc-robot-dev, matching ./k3d_up.sh)
set -euo pipefail

CLUSTER="${DC_K3D_CLUSTER:-dc-robot-dev}"

if ! command -v k3d >/dev/null; then
  echo "k3d is required: https://k3d.io/#installation" >&2
  exit 1
fi

echo "[k3d-down $(date -u +%H:%M:%S)] deleting cluster $CLUSTER"
k3d cluster delete "$CLUSTER"
