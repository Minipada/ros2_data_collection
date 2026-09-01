#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Loads one or more images into the #452 kind cluster without a registry — same
# no-registry acceptance criterion #451's k3d loop states for its own images. `kind load
# docker-image` reads from the *Docker* image store, which this repo's own podman-built
# images never populate (CLAUDE.md "Containers: Podman, not Docker"), so this goes
# through a tar instead: `podman save` -> `kind load image-archive`. Avoiding a
# cluster-side image pull also means the Pods below never need a ghcr.io pull secret.
#
#   ./load_images.sh <cluster-name> <image-ref> [<image-ref> ...]
set -euo pipefail

CLUSTER_NAME="$1"
shift

log() { echo "[kind-load-images $(date -u +%H:%M:%S)] $*"; }

for IMAGE in "$@"; do
  log "pulling $IMAGE (podman)"
  podman image exists "$IMAGE" || podman pull "$IMAGE"

  TAR="$(mktemp -t kind-image.XXXXXX.tar)"
  log "saving $IMAGE -> $TAR"
  podman save -o "$TAR" "$IMAGE"

  log "loading $TAR into cluster $CLUSTER_NAME"
  kind load image-archive "$TAR" --name "$CLUSTER_NAME"
  rm -f "$TAR"
done
