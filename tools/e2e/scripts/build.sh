#!/bin/bash
# Builds tools/e2e/Containerfile — the shared DC workspace image both Jazzy CI
# (.github/workflows/ci-jazzy.yaml, runs `colcon test` directly against this) and the
# E2E harness (tools/e2e/Containerfile.e2e, a thin `FROM` layer adding the harness's
# own runtime bits — see tools/e2e/scripts/run.sh) build from. No separate CI-only
# build script (see CLAUDE.md "Containers: Podman, not Docker").
#
# Env vars (all optional):
#   IMAGE_TAG   image tag to build (default dc-workspace:latest)
#   CCOV        "true" to build with C++ coverage instrumentation (default false)
#   CACHE_REF   a registry ref (e.g. ghcr.io/<repo>/dc-e2e-cache) to use as a
#               podman --cache-from/--cache-to target. podman's build cache is
#               otherwise local-only and doesn't survive a fresh machine/runner —
#               CI sets this so a cold GitHub-hosted runner still gets warm layers
#               for the (rarely-changing) apt/toolchain RUN. Omit for a plain local
#               build with no registry round-trip.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

IMAGE_TAG="${IMAGE_TAG:-dc-workspace:latest}"
CCOV="${CCOV:-false}"

# The repo's .dockerignore is an "ignore everything, whitelist package.xml" allowlist
# meant for a deps-only image — this build needs every source file. `--ignorefile`
# does *not* override .dockerignore (verified empirically — it only supplements it),
# so it has to be moved aside, matching docker.yaml's own `source`/`source-sim` jobs'
# `rm .dockerignore` precedent for the same reason. Trap-guarded so it's restored even
# on failure.
DOCKERIGNORE="$REPO_ROOT/.dockerignore"
mv "$DOCKERIGNORE" "$DOCKERIGNORE.e2e-bak"
trap 'if [ -f "$DOCKERIGNORE.e2e-bak" ]; then mv -f "$DOCKERIGNORE.e2e-bak" "$DOCKERIGNORE"; fi' EXIT

ARGS=(
  build --layers
  # Podman defaults to OCI image format, which silently ignores the Containerfile's
  # `SHELL` instruction ("SHELL is not supported for OCI image format" — a warning,
  # not a failure, easy to miss); --format docker makes it take effect, so RUN steps
  # using a pipe actually get -o pipefail instead of always "succeeding" even if an
  # earlier stage in the pipe failed.
  --format docker
  --build-arg "APT_CACHEBUST=$(date -u +%Y-%m-%d)"
  --build-arg "CCOV=$CCOV"
  -t "$IMAGE_TAG"
  -f "$REPO_ROOT/tools/e2e/Containerfile"
)
if [ -n "${CACHE_REF:-}" ]; then
  ARGS+=(--cache-from "$CACHE_REF" --cache-to "$CACHE_REF")
fi
ARGS+=("$REPO_ROOT")

podman "${ARGS[@]}"
