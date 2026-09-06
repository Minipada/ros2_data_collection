# shellcheck shell=bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Shared setup for build_doc.sh and serve_doc.sh: the toolchain image, the run_in_image
# helper, and the two generated-content steps (strictdoc export, ADR pages) both scripts
# need before mdbook runs. Sourced, not executed — keeps the two entry points from
# drifting apart on how the book's inputs get prepared.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
IMAGE_TAG="${IMAGE_TAG:-dc-doc:local}"
ENGINE="${ENGINE:-podman}"

build_doc_image() {
    echo "==> Building the docs toolchain image ${IMAGE_TAG}"
    BUILD_ARGS=(build --layers -t "${IMAGE_TAG}" -f "${REPO_ROOT}/containers/doc/Containerfile")
    if [ -n "${CACHE_REF:-}" ]; then
        BUILD_ARGS+=(--cache-from "${CACHE_REF}" --cache-to "${CACHE_REF}")
    fi
    BUILD_ARGS+=("${REPO_ROOT}/containers/doc")
    "${ENGINE}" "${BUILD_ARGS[@]}"
}

run_in_image() {
    "${ENGINE}" run --rm \
        -v "${REPO_ROOT}:/ws:z" \
        --workdir "/ws${1}" \
        "${IMAGE_TAG}" \
        "${@:2}"
}

generate_doc_inputs() {
    echo "==> Exporting the strictdoc requirements"
    run_in_image "" strictdoc export requirements \
        --experimental-enable-file-traceability \
        --enable-mathjax \
        --format=html \
        --output-dir doc/src/dc/requirements \
        --project-title "ROS 2 Data Collection"

    echo "==> Generating ADR pages"
    run_in_image "" python3 tools/ci/pre-commit/generate_adr_pages.py
}
