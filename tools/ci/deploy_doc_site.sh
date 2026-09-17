#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

set -euo pipefail

# Publish one distro's docs section to the shared GitHub Pages site (the gh-pages branch).
#
# Each distro branch runs this on its own push (#534): the script replaces only this
# distro's /<distro>/ subtree plus the root landing page and 404, and leaves every sibling
# distro's section untouched — there is no step that composes the site from all distros.
# The site root is owned by this publisher (landing page, 404, .nojekyll, the distro
# sections); anything else at the root is stale from the pre-#534 monolithic book and is
# removed, so the first publish on each branch retires it.
#
# Authentication comes from the checkout that ran this script: actions/checkout persists
# job credentials in the workspace repo's git config, which this script's fetch and push
# inherit.
#
# Concurrent publishes from two distro branches can race on one gh-pages branch; the push
# is retried against the fresh tip, and since each attempt rebuilds the same deterministic
# tree the retry converges.
#
# Environment:
#   DISTRO        required — this build's distro section (rolling, lyrical or jazzy;
#                 same list as doc/js/distro.js)
#   SITE_DIR      staging dir laid out by build_doc.sh with DISTRO set (default
#                 doc/book/html): must contain <DISTRO>/, index.html and 404.html
#   PAGES_BRANCH  Pages branch (default gh-pages)
#   PAGES_REMOTE  git remote to fetch/push that branch from (default origin; a local
#                 remote name lets a dry run target a scratch repo)

REPO_ROOT="$(git rev-parse --show-toplevel)"
DISTRO="${DISTRO:?DISTRO is required (rolling, lyrical or jazzy)}"
SITE_DIR="${SITE_DIR:-doc/book/html}"
case "$SITE_DIR" in
    /*) ;;
    *) SITE_DIR="$REPO_ROOT/$SITE_DIR" ;;
esac
PAGES_BRANCH="${PAGES_BRANCH:-gh-pages}"
PAGES_REMOTE="${PAGES_REMOTE:-origin}"

for required in "$DISTRO" index.html 404.html; do
    if [ ! -e "$SITE_DIR/$required" ]; then
        echo "error: $SITE_DIR/$required is missing — run build_doc.sh with DISTRO=$DISTRO first" >&2
        exit 1
    fi
done

SOURCE_SHA="$(git -C "$REPO_ROOT" rev-parse --short HEAD)"
WORKTREE="$(mktemp -d)"

# Keep in sync with doc/js/distro.js — the site's distro list. Sibling sections are
# preserved; the root is otherwise cleaned down to the publisher's own files.
DISTROS=(rolling lyrical jazzy)

CLEAN_ARGS=(-not -name .git -not -name .nojekyll -not -name CNAME)
for id in "${DISTROS[@]}"; do
    CLEAN_ARGS+=(-not -name "$id")
done

reset_worktree() {
    git -C "$REPO_ROOT" worktree remove --force "$WORKTREE" 2>/dev/null || true
    git -C "$REPO_ROOT" worktree prune
    rm -rf "$WORKTREE"
}
trap reset_worktree EXIT

publish_attempt() {
    reset_worktree

    if git -C "$REPO_ROOT" fetch "$PAGES_REMOTE" "$PAGES_BRANCH"; then
        git -C "$REPO_ROOT" worktree add --detach "$WORKTREE" \
            "refs/remotes/$PAGES_REMOTE/$PAGES_BRANCH"
    else
        echo "==> No existing $PAGES_BRANCH branch, starting the site from scratch"
        git -C "$REPO_ROOT" worktree add --orphan -b "dc-docs-$PAGES_BRANCH" "$WORKTREE"
    fi

    # `set -e` again because errexit is suspended in this function's `until` condition
    # context, and a silent cp failure would publish a section with no landing page.
    (
        set -e
        cd "$WORKTREE"
        find . -mindepth 1 -maxdepth 1 "${CLEAN_ARGS[@]}" -exec rm -rf {} +
        rm -rf "./$DISTRO"
        cp -a "$SITE_DIR/$DISTRO" "./$DISTRO"
        cp -a "$SITE_DIR/index.html" "$SITE_DIR/404.html" .
        touch .nojekyll
    )

    if [ -z "$(git -C "$WORKTREE" status --porcelain)" ]; then
        echo "==> Site already up to date, nothing to publish"
        return 0
    fi

    git -C "$WORKTREE" add -A
    git -C "$WORKTREE" \
        -c user.name="github-actions[bot]" \
        -c user.email="41898282+github-actions[bot]@users.noreply.github.com" \
        -c core.hooksPath=/dev/null \
        commit -m "docs($DISTRO): publish docs section from $SOURCE_SHA"

    if ! git -C "$WORKTREE" push "$PAGES_REMOTE" "HEAD:refs/heads/$PAGES_BRANCH"; then
        echo "==> Push rejected (a concurrent publish moved $PAGES_BRANCH), retrying"
        return 1
    fi
}

ATTEMPT=1
until publish_attempt; do
    if [ "$ATTEMPT" -ge 3 ]; then
        echo "error: giving up after $ATTEMPT attempts" >&2
        exit 1
    fi
    ATTEMPT=$((ATTEMPT + 1))
    sleep 10
done

echo "==> Published $SITE_DIR to $PAGES_REMOTE $PAGES_BRANCH as the /$DISTRO/ section"
