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
#   DISTRO      Publish this build as one distro's section of the versioned docs site
#               (issue #534): the book lands in doc/book/html/<DISTRO>/ with site-url
#               /<DISTRO>/, and the landing page (doc/landing/) is staged at the site
#               root — the layout tools/ci/deploy_doc_site.sh publishes. Unset (the
#               pre-commit hook, serve_doc.sh) builds the plain single-site book at
#               doc/book/html as before.
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
if [ -n "${DISTRO:-}" ]; then
    # Site-relative URLs in the built 404 page; everything else already resolves
    # through per-page relative paths. `podman run` doesn't forward host env, so the
    # override rides in on the command line.
    run_in_image "/doc" env "MDBOOK_OUTPUT__HTML__SITE_URL=/${DISTRO}/" mdbook build
else
    run_in_image "/doc" mdbook build
fi

if [ -n "${DISTRO:-}" ]; then
    echo "==> Staging the ${DISTRO} section"
    mv doc/book/html "doc/book/html-${DISTRO}"
    mkdir doc/book/html
    mv "doc/book/html-${DISTRO}" "doc/book/html/${DISTRO}"
    cp doc/landing/index.html doc/landing/404.html doc/book/html/
fi

echo "==> Documentation built in doc/book/html"
