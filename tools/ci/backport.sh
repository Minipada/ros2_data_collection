#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

set -euo pipefail

# Cherry-pick a merged rolling PR onto the distro branches its backport:* labels name.
#
# One script for every place it runs (CLAUDE.md "one script for CI and local dev"): the
# `backport` workflow on a merged PR, its dispatch dry run, and a developer's terminal.
# A cherry-pick that conflicts still produces an open PR — the conflicted files are
# committed with their markers and listed in the PR body — never a silent failure.
# PRs created with the workflow's own GITHUB_TOKEN don't start CI; close and reopen a
# backport PR to trigger it.
#
# Usage:
#   tools/ci/backport.sh <pr-number> [--dry-run]
#
# Environment:
#   DRY_RUN             true = plan only (same as --dry-run; the workflow's dispatch input)
#   GITHUB_REPOSITORY   owner/name (set by Actions); else parsed from the origin remote
#   GITHUB_STEP_SUMMARY written to as well when set (Actions run summary)

usage() {
    echo "usage: $0 <pr-number> [--dry-run]" >&2
    exit 1
}

PR="${1:-}"
[[ "$PR" =~ ^[0-9]+$ ]] || usage
DRY_RUN="${DRY_RUN:-false}"
case "${2:-}" in
"") ;;
--dry-run) DRY_RUN=true ;;
*) usage ;;
esac
[[ $# -le 2 ]] || usage

DEVELOP_BRANCH=rolling
BACKPORT_PREFIX=backport

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

if [[ -n "${GITHUB_REPOSITORY:-}" ]]; then
    REPO="$GITHUB_REPOSITORY"
else
    REMOTE_URL="$(git remote get-url origin)"
    REPO="${REMOTE_URL#*github.com[:/]}"
    REPO="${REPO%.git}"
    [[ "$REPO" != "$REMOTE_URL" && "$REPO" == */* ]] ||
        {
            echo "error: cannot derive owner/repo from $REMOTE_URL (set GITHUB_REPOSITORY)" >&2
            exit 1
        }
fi
GH=(gh --repo "$REPO")

say() {
    echo "$@"
    if [[ -n "${GITHUB_STEP_SUMMARY:-}" ]]; then
        echo "$@" >>"$GITHUB_STEP_SUMMARY"
    fi
}

die() {
    echo "error: $*" >&2
    exit 1
}

PR_JSON="$("${GH[@]}" pr view "$PR" --json number,title,url,state,baseRefName,labels,commits)"
PR_TITLE="$(jq -r .title <<<"$PR_JSON")"
PR_STATE="$(jq -r .state <<<"$PR_JSON")"
PR_MERGED="$([[ "$PR_STATE" == "MERGED" ]] && echo true || echo false)"
PR_BASE="$(jq -r .baseRefName <<<"$PR_JSON")"

# Targets: the branches the PR's backport:<branch> labels name.
JQ_TARGETS=".labels[].name | select(startswith(\"$BACKPORT_PREFIX:\")) | sub(\"$BACKPORT_PREFIX:\"; \"\")"
mapfile -t TARGETS < <(jq -r "$JQ_TARGETS" <<<"$PR_JSON" | sort -u)
mapfile -t COMMITS < <(jq -r '.commits[] | .oid + " " + .messageHeadline' <<<"$PR_JSON")

echo "PR #${PR}: ${PR_TITLE}"
echo "Base branch: ${PR_BASE}"
if [[ ${#TARGETS[@]} -eq 0 ]]; then
    echo "No ${BACKPORT_PREFIX}:* label — nothing to backport."
    exit 0
fi
echo "Targets: ${TARGETS[*]}"
[[ ${#COMMITS[@]} -gt 0 ]] || die "PR #${PR} has no commits"
echo "Commits to cherry-pick (${#COMMITS[@]}):"
for commit in "${COMMITS[@]}"; do
    echo "  - $commit"
done

# Distros are branches like any other: refuse a target that doesn't exist remotely
# instead of pushing a branch a backport:<distro> typo invented.
for target in "${TARGETS[@]}"; do
    [[ "$target" != "$DEVELOP_BRANCH" ]] ||
        die "${BACKPORT_PREFIX}:${target} would backport onto the development branch itself"
    git ls-remote --exit-code --heads origin "$target" >/dev/null ||
        die "branch '${target}' does not exist on origin; fix the ${BACKPORT_PREFIX}:${target} label"
done

if [[ "$DRY_RUN" == "true" ]]; then
    echo ""
    echo "Dry run — this is the plan, nothing was created:"
    for target in "${TARGETS[@]}"; do
        echo "  → ${target}: branch ${BACKPORT_PREFIX}-${PR}-to-${target} from origin/${target}"
        echo "    cherry-pick ${#COMMITS[@]} commit(s), open PR 'backport: #${PR} to ${target}'"
    done
    exit 0
fi

# From here on the run creates things, so it must be a merge into the development branch:
# the distro branches' own PRs carry no backport labels that should recurse.
[[ "$PR_MERGED" == "true" ]] || die "PR #${PR} is ${PR_STATE}, not merged"
[[ "$PR_BASE" == "$DEVELOP_BRANCH" ]] ||
    die "PR #${PR} targets '${PR_BASE}', not '${DEVELOP_BRANCH}'"

# The PR's own commits live on its (usually deleted) head branch; refs/pull/<N>/head
# still points at them. Refresh everything the cherry-picks need.
git fetch --no-tags origin \
    "${DEVELOP_BRANCH}:refs/remotes/origin/${DEVELOP_BRANCH}" \
    "refs/pull/${PR}/head:refs/remotes/pull/${PR}/head"
for target in "${TARGETS[@]}"; do
    git fetch --no-tags origin "${target}:refs/remotes/origin/${target}"
done

# Hooks must not fire on the bot's cherry-pick commits (a conflict carries markers a
# merge-conflict hook would reject). Identity only needs setting in Actions.
GIT_ID=(-c core.hooksPath=/dev/null -c commit.gpgsign=false)
if [[ -n "${GITHUB_ACTIONS:-}" ]]; then
    GIT_ID+=(-c user.name=github-actions[bot] -c user.email=41898282+github-actions[bot]@users.noreply.github.com)
fi

WORKTREE="$(mktemp -d)"
cleanup() {
    git worktree remove --force "$WORKTREE" 2>/dev/null || true
    git worktree prune
    rm -rf "$WORKTREE"
}
trap cleanup EXIT

BACKPORT_PRS=()

for target in "${TARGETS[@]}"; do
    BRANCH="${BACKPORT_PREFIX}-${PR}-to-${target}"

    # Idempotent re-runs: an open backport PR means this already happened.
    if [[ -n "$("${GH[@]}" pr list --head "$BRANCH" --base "$target" --state open --json number --jq '.[].number')" ]]; then
        say "==> #${PR}: backport to ${target} already has an open PR, skipping"
        continue
    fi

    git worktree add --detach "$WORKTREE" "refs/remotes/origin/$target" >/dev/null
    git -C "$WORKTREE" switch -C "$BRANCH"

    declare -A CONFLICT_FILES=()
    for commit in "${COMMITS[@]}"; do
        read -r sha subject <<<"$commit"
        if git -C "$WORKTREE" "${GIT_ID[@]}" cherry-pick "$sha"; then
            continue
        fi
        # Conflicts are not an error: commit them with their markers so the PR shows
        # them; a pick that ends up empty (change already there) is skipped instead.
        if git -C "$WORKTREE" diff --name-only --diff-filter=U | grep -q .; then
            while IFS= read -r file; do
                CONFLICT_FILES["$file"]=1
            done < <(git -C "$WORKTREE" diff --name-only --diff-filter=U)
            git -C "$WORKTREE" add -A
            if git -C "$WORKTREE" diff --cached --quiet; then
                git -C "$WORKTREE" -c core.editor=true "${GIT_ID[@]}" cherry-pick --skip
            else
                git -C "$WORKTREE" -c core.editor=true "${GIT_ID[@]}" cherry-pick --continue
            fi
        else
            git -C "$WORKTREE" -c core.editor=true "${GIT_ID[@]}" cherry-pick --skip
        fi
    done

    git -C "$WORKTREE" push --force origin "HEAD:refs/heads/$BRANCH"

    BODY="Backport of #${PR} to \`${target}\`, cherry-picked by the [Backport workflow](${GITHUB_SERVER_URL:-https://github.com}/${GITHUB_REPOSITORY:-$REPO}/actions/workflows/backport.yml).

Commits:
"
    for commit in "${COMMITS[@]}"; do
        read -r sha subject <<<"$commit"
        BODY+="\`${sha:0:9}\` ${subject}
"
    done
    if [[ ${#CONFLICT_FILES[@]} -gt 0 ]]; then
        BODY+="
⚠️ These files conflicted during the cherry-pick and are committed **with conflict markers** — resolve them before merging:
"
        for file in "${!CONFLICT_FILES[@]}"; do
            BODY+="- \`${file}\`
"
        done
    fi
    BODY+="
This PR does not start CI on its own (workflows don't run for PRs created with a workflow's token); close and reopen it to trigger the checks."

    PR_URL="$("${GH[@]}" pr create \
        --base "$target" \
        --head "$BRANCH" \
        --title "backport: #${PR} to ${target} — ${PR_TITLE}" \
        --body "$BODY")"
    BACKPORT_PRS+=("$PR_URL")
    say "==> #${PR}: opened $PR_URL"
    if [[ ${#CONFLICT_FILES[@]} -gt 0 ]]; then
        say "    ⚠️ with conflicts in: ${!CONFLICT_FILES[*]}"
    fi

    git worktree remove --force "$WORKTREE" 2>/dev/null || true
    git worktree prune
done

if [[ ${#BACKPORT_PRS[@]} -gt 0 ]]; then
    COMMENT="Backport PRs: ${BACKPORT_PRS[*]}"
    "${GH[@]}" pr comment "$PR" --body "$COMMENT" >/dev/null
    say "==> #${PR}: ${COMMENT}"
else
    say "==> #${PR}: nothing to do (all backports already have open PRs)"
fi
