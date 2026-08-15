#!/bin/bash
# Builds tools/e2e/Containerfile — the shared DC workspace image both CI
# (.github/workflows/ci.yaml's build-workspace job, which also runs `colcon test`
# as part of this same build — see the Containerfile's `workspace` stage) and the
# E2E harness (tools/e2e/Containerfile.e2e, a thin `FROM` layer adding the harness's
# own runtime bits — see tools/e2e/scripts/run.sh) build from. CI and local dev run
# this exact script the exact same way — no separate CI-only build script (see
# CLAUDE.md "Containers: Podman, not Docker").
#
# --network host: the workspace stage's `colcon test` needs to reach the
# Postgres/RustFS test-dependency stores (tools/e2e/compose.test.yaml) dc_bridge's
# store-backed tests hard-fail without — the caller is responsible for having them
# already running on the host before calling this script.
#
# Env vars (all optional):
#   IMAGE_TAG      image tag to build (default dc-workspace:latest)
#   CCOV           "true" to build with C++ coverage instrumentation (default false)
#   TARGET         stage to build, e.g. `toolchain` — omit to build the full
#                  `workspace` stage as today
#   CACHE_REF      a registry ref (e.g. ghcr.io/<repo>/dc-e2e-cache) to use as a
#                  podman --cache-from/--cache-to target for LAYER caching (the
#                  rarely-changing apt/toolchain/aws_sdk_vendor layers). podman's
#                  layer cache is otherwise local-only and doesn't survive a fresh
#                  machine/runner. Omit for a plain local build with no registry
#                  round-trip.
#   BUILDAH_TMPDIR a directory to use as $TMPDIR for this build, so the workspace
#                  stage's `RUN --mount=type=cache` ccache mount — which buildah
#                  stores under $TMPDIR/buildah-cache/<id>, entirely separate from
#                  --cache-from/--cache-to's registry export (verified: --cache-to
#                  does not carry cache-mount content, only layers) — lands
#                  somewhere the caller can persist across runs (see ci.yaml's
#                  actions/cache step) instead of the default $TMPDIR (/tmp on a
#                  GitHub-hosted runner, gone once the job ends). Omit for a plain
#                  local build using the system default.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

IMAGE_TAG="${IMAGE_TAG:-dc-workspace:latest}"
CCOV="${CCOV:-false}"

if [ -n "${BUILDAH_TMPDIR:-}" ]; then
  mkdir -p "$BUILDAH_TMPDIR"
  export TMPDIR="$BUILDAH_TMPDIR"
fi

# The repo's .dockerignore is an "ignore everything, whitelist package.xml" allowlist
# meant for a deps-only image — this build needs every source file. `--ignorefile`
# does *not* override .dockerignore (verified empirically — it only supplements it),
# so it has to be moved aside for the duration of the build. Trap-guarded so it's
# restored even on failure.
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
  --network host
  --build-arg "APT_CACHEBUST=$(date -u +%Y-%m-%d)"
  --build-arg "CCOV=$CCOV"
  -t "$IMAGE_TAG"
  -f "$REPO_ROOT/tools/e2e/Containerfile"
)
if [ -n "${TARGET:-}" ]; then
  ARGS+=(--target "$TARGET")
fi
if [ -n "${CACHE_REF:-}" ]; then
  ARGS+=(--cache-from "$CACHE_REF" --cache-to "$CACHE_REF")
fi
ARGS+=("$REPO_ROOT")

podman "${ARGS[@]}"
