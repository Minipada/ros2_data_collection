#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Schema-validates Kubernetes manifests with kubeconform (#450), run via Podman —
# CLAUDE.md "Containers: Podman, not Docker" — rather than a locally installed
# kubeconform binary. Called by the `kubeconform` pre-commit hook (pre-commit passes
# changed-file paths as args) and directly for ad hoc checks:
#
#   ./tools/ci/pre-commit/kubeconform.sh deploy/robot/kubernetes/robot-pod.yaml
#
# Env vars:
#   IMAGE_TAG  kubeconform image tag (default pinned below; bump deliberately).
set -euo pipefail

IMAGE_TAG="${IMAGE_TAG:-v0.6.7}"
IMAGE="ghcr.io/yannh/kubeconform:${IMAGE_TAG}-alpine"

if [ "$#" -eq 0 ]; then
  echo "usage: $0 <manifest.yaml> [more-manifests.yaml...]" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

# Mount the repo read-only and pass repo-relative paths through, so kubeconform's
# error output matches what a developer sees in their editor. Accepts both absolute
# paths and paths relative to the current directory (pre-commit passes the latter,
# relative to REPO_ROOT since that's where prek runs hooks from).
args=()
for f in "$@"; do
  args+=("/repo/$(realpath --relative-to="$REPO_ROOT" "$f")")
done

exec podman run --rm -v "$REPO_ROOT:/repo:ro" "$IMAGE" \
  -strict -summary "${args[@]}"
