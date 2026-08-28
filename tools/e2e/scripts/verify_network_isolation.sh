#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Builds tools/e2e/Containerfile's `vendor-network-check` stage (#423) — a harness
# proving aws_sdk_vendor's build-time network fetch (ExternalProject_Add's
# GIT_REPOSITORY) is isolated to its own `colcon build` step via that stage's
# `RUN --network=none` (a per-instruction Buildah/Podman flag independent of this
# script's own build.sh's top-level `--network host`), not smeared together with
# rosdep/apt's own allowed network use. vector_vendor's own build-time fetch was the
# same kind of check until #424 replaced it with a checked-in binary (since split into
# its own repo, fetched via `vcs import` before this same stage's isolated build step —
# see docs/adr/0002-vector-as-default-shipper.md).
#
# aws_sdk_vendor builds from a flattened, plain-committed source tree (ADR-0012,
# #425) — no network I/O — so this build is expected to succeed.
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
