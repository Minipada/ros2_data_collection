#!/bin/bash

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

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
IMAGE_TAG="${IMAGE_TAG:-dc-doc:local}"
ENGINE="${ENGINE:-podman}"

echo "==> Building the docs toolchain image ${IMAGE_TAG}"
"${ENGINE}" build -t "${IMAGE_TAG}" -f "${REPO_ROOT}/containers/doc/Containerfile" \
    "${REPO_ROOT}/containers/doc"

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
