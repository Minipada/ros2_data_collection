#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

set -euo pipefail

# Live-reloading dev server for the documentation site: same toolchain image and the same
# generated inputs as build_doc.sh (strictdoc export, ADR pages), but `mdbook serve`
# instead of a one-shot `mdbook build` — editing any doc/src/**/*.md live-reloads the
# browser. Editing docs/adr/*.md doesn't live-reload on its own: re-run
# tools/ci/pre-commit/generate_adr_pages.py to regenerate doc/src/dc/adr/, which mdbook
# then picks up like any other doc/src change.
#
# Environment: same as build_doc.sh, plus PORT (default 3000, published to the host).

# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_doc_common.sh"
PORT="${PORT:-3000}"

build_doc_image
generate_doc_inputs

# `mdbook serve`'s own first build (pinned 0.4.52) skips copying the theme's static assets
# (css/, book.js, ...) when doc/book/ doesn't already exist -- a plain `mdbook build` doesn't
# have this problem, so seed doc/book/html with one before handing off to `serve`, which then
# preserves it across its own incremental rebuilds.
echo "==> Building the book once to seed static assets"
run_in_image "/doc" mdbook build

echo "==> Serving the book at http://127.0.0.1:${PORT} (Ctrl-C to stop)"
"${ENGINE}" run --rm -it \
    -v "${REPO_ROOT}:/ws:z" \
    --workdir /ws/doc \
    -p "${PORT}:3000" \
    "${IMAGE_TAG}" \
    mdbook serve --hostname 0.0.0.0 --port 3000
