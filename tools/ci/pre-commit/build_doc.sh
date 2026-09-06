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
# preprocessor ABI is not stable across minor versions). For an editing loop with live
# reload instead of a one-shot build, see serve_doc.sh.
#
# Output: doc/book/html (site), doc/src/dc/requirements/html (requirements export) and
# doc/src/dc/adr/ (ADR pages mirrored from docs/adr/) — both generated before mdbook runs
# so mdbook picks them up as part of the book.
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

# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_doc_common.sh"

build_doc_image
generate_doc_inputs

echo "==> Building the book"
run_in_image "/doc" mdbook build

echo "==> Documentation built in doc/book/html"
