#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# The deployment/rendering trees pin the Vector image as literals — compose.yaml, the
# quadlet, the Kubernetes pod, the kind manifests, the Helm values, and
# compose.split.yaml's ${VECTOR_VERSION:-...} default — because neither Quadlet nor
# plain YAML can read ros2_data_collection.repos, where the canonical pin lives. This
# check fails when any of those copies stops matching the pin, so a bump that misses
# one surfaces in CI instead of at deploy time on a robot (#484). Runs standalone:
#   ./tools/ci/check_vector_pin.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# shellcheck disable=SC1091
source "$REPO_ROOT/tools/e2e/scripts/lib/version.sh"
pin="$(vector_version)"

# `-debian` semver tags are Vector's alone in these trees (dc-ros/dc-uploader pin
# shas or branch refs), so any literal not equal to the pin is a stale copy.
stale_tags="$(grep -rn --include='*.yaml' --include='*.yml' --include='*.container' \
  -E '[0-9]+\.[0-9]+\.[0-9]+-debian' "$REPO_ROOT/deploy" "$REPO_ROOT/tools" \
  | grep -v "${pin}-debian" || true)"

stale_defaults="$(grep -rn -E 'VECTOR_VERSION:-[0-9]+\.[0-9]+\.[0-9]+' "$REPO_ROOT/tools" \
  | grep -v "VECTOR_VERSION:-${pin}" || true)"

if [[ -n "$stale_tags" || -n "$stale_defaults" ]]; then
  {
    echo "Vector version literals out of step with the ros2_data_collection.repos pin (v${pin}):"
    echo "$stale_tags"
    echo "$stale_defaults"
    echo "Update these to ${pin}-debian (or bump them together with the .repos pin)."
  } >&2
  exit 1
fi
echo "All Vector version literals match the ros2_data_collection.repos pin (v${pin})."
