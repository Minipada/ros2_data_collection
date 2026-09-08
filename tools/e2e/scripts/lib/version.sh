# shellcheck shell=bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# The one reader of the canonical Vector version pin, ros2_data_collection.repos'
# vector_vendor entry (#484). Its own file, not part of lib/harness.sh, so the
# limits/load and release scripts can source it without pulling in that e2e-only
# skeleton; harness.sh sources it too, for the scenario scripts. Sourced, never
# executed, and harmless to source twice.

VERSION_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION_REPO_ROOT="$(cd "$VERSION_LIB_DIR/../../../.." && pwd)"

# Prints the pinned version without its leading "v" ("0.57.0") — the form the
# timberio/vector image tags carry. -A3 scopes the parse to vector_vendor's own
# block: aws_sdk_vendor's `version: v...` line would otherwise match too.
vector_version() {
  local version
  version="$(grep -A3 'vector_vendor:' \
    "$VERSION_REPO_ROOT/ros2_data_collection.repos" | grep -oP 'version: v\K[0-9.]+')" || {
    echo "vector_version: cannot read the vector_vendor pin in $VERSION_REPO_ROOT/ros2_data_collection.repos" >&2
    return 1
  }
  echo "$version"
}
