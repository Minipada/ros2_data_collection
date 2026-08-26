#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Builds tools/e2e/Containerfile's `vendor-network-check` stage (#423) — a harness
# proving that aws_sdk_vendor's and vector_vendor's build-time network fetches
# (ExternalProject_Add's GIT_REPOSITORY, file(DOWNLOAD ...)) are isolated to their own
# `colcon build` step via that stage's `RUN --network=none` (a per-instruction
# Buildah/Podman flag independent of this script's own build.sh's top-level
# `--network host`), not smeared together with rosdep/apt's own allowed network use.
#
# Today, before #424 (vector_vendor prebuilt binaries) and #425 (aws_sdk_vendor
# flattened source) replace those git-clone/download calls with vendored/prebuilt
# sources, this build is EXPECTED TO FAIL with a network-access error — that failure
# is the proof the isolation seam works, not a bug in this script. Once #424/#425
# land, this same command should start succeeding.
#
# Env vars (all optional, forwarded to build.sh):
#   IMAGE_TAG      image tag to build (default dc-vendor-network-check:latest)
#   CACHE_REF      see build.sh — shares the toolchain-base apt layer's cache with the
#                  main build-workspace build
#   BUILDAH_TMPDIR see build.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export IMAGE_TAG="${IMAGE_TAG:-dc-vendor-network-check:latest}"
export TARGET=vendor-network-check

"$SCRIPT_DIR/build.sh"
