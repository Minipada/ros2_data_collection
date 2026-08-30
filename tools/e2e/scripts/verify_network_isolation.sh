#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Builds tools/e2e/Containerfile's `vendor-network-check` stage (#423) — a harness
# proving aws_sdk_vendor's build-time network fetch (ament_vendor()'s `vcs import` of
# aws-sdk-cpp) and vector_vendor's build-time network fetch (checksum-pinned
# `file(DOWNLOAD ...)` of the Vector release tarball) are each isolated to their own
# `colcon build` step via that stage's `RUN --network=none` (a per-instruction
# Buildah/Podman flag independent of this script's own build.sh's top-level
# `--network host`), not smeared together with rosdep/apt's own allowed network use.
#
# This build is EXPECTED TO FAIL with a network-access error, permanently, for both
# packages — aws_sdk_vendor fetches aws-sdk-cpp live via ament_vendor() by design
# (docs/adr/0012), and vector_vendor fetches its release tarball live by design
# (docs/adr/0002-vector-as-default-shipper.md's reversal, #435), matching what the real
# ROS buildfarm's binarydeb jobs actually allow (docker run --net=host). The failure
# here is the proof the isolation seam works, not a bug in this script.
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
