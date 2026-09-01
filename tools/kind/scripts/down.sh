#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Tears down the #452 kind cluster. Idempotent: `kind delete cluster` on a name that
# doesn't exist is a no-op, not an error.
set -euo pipefail

CLUSTER_NAME="${DC_KIND_CLUSTER:-dc-kind}"

echo "[kind-down $(date -u +%H:%M:%S)] deleting cluster $CLUSTER_NAME"
kind delete cluster --name "$CLUSTER_NAME"
