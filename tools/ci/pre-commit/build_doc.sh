#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

set -euo pipefail

# Build the documentation site (mdbook) and the strictdoc requirements export.
#
# Called by both the `build-doc` pre-commit hook and .github/workflows/doc.yaml, so a
# local docs build and the CI docs build run the same toolchain — there is no CI-only
# docs script. The toolchain lives in a Podman image built from
# containers/doc/Containerfile with pinned mdbook/strictdoc versions (mdbook's
# preprocessor ABI is not stable across minor versions).
#
# Output: doc/book/html (site) and doc/src/dc/requirements/html (requirements export,
# generated before mdbook runs so mdbook copies it into the site).
#
# Environment:
#   IMAGE_TAG   Tag for the builder image (default dc-doc:local; CI passes dc-doc:<sha>)
#   ENGINE      Container engine (default podman)
#   CACHE_REF   a registry ref (e.g. ghcr.io/<repo>/dc-doc-cache) to use as a podman
#               --cache-from/--cache-to target. podman's build cache is otherwise
#               local-only and doesn't survive a fresh machine/runner — CI sets this so a
#               cold GitHub-hosted runner still gets warm layers for the rarely-changing
#               rust toolchain + cargo-installed mdbook/strictdoc RUN steps. Omit for a
#               plain local build with no registry round-trip (the default).

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
IMAGE_TAG="${IMAGE_TAG:-dc-doc:local}"
ENGINE="${ENGINE:-podman}"

echo "==> Building the docs toolchain image ${IMAGE_TAG}"
BUILD_ARGS=(build --layers -t "${IMAGE_TAG}" -f "${REPO_ROOT}/containers/doc/Containerfile")
if [ -n "${CACHE_REF:-}" ]; then
    BUILD_ARGS+=(--cache-from "${CACHE_REF}" --cache-to "${CACHE_REF}")
fi
BUILD_ARGS+=("${REPO_ROOT}/containers/doc")
"${ENGINE}" "${BUILD_ARGS[@]}"

run_in_image() {
    "${ENGINE}" run --rm \
        -v "${REPO_ROOT}:/ws:z" \
        --workdir "/ws${1}" \
        "${IMAGE_TAG}" \
        "${@:2}"
}

echo "==> Exporting the strictdoc requirements"
run_in_image "" strictdoc export requirements \
    --experimental-enable-file-traceability \
    --enable-mathjax \
    --format=html \
    --output-dir doc/src/dc/requirements \
    --project-title "ROS 2 Data Collection"

echo "==> Building the book"
run_in_image "/doc" mdbook build

echo "==> Documentation built in doc/book/html"
