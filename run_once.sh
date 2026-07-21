#!/bin/bash
set -e
set -o pipefail

AGENT="${RUN_ONCE_AGENT:-claude}"

usage() {
	echo "Usage: $0 [--agent claude|codex]"
	echo "Example: $0"
	echo "Example: $0 --agent codex"
	echo "Example: RUN_ONCE_AGENT=codex $0"
	echo ""
	echo "Picks the oldest open 'ready-for-agent' issue with no open PR and no open"
	echo "'Blocked by' dependency, does the work in an isolated git worktree, and"
	echo "opens a PR that closes it."
}

while [[ $# -gt 0 ]]; do
	case "$1" in
	--agent)
		if [[ -z "${2:-}" ]]; then
			echo "ERROR: --agent requires claude or codex"
			exit 1
		fi
		AGENT="$2"
		shift 2
		;;
	--agent=*)
		AGENT="${1#--agent=}"
		shift
		;;
	-h | --help)
		usage
		exit 0
		;;
	claude | codex)
		AGENT="$1"
		shift
		;;
	*)
		usage
		exit 1
		;;
	esac
done

case "$AGENT" in
claude | codex) ;;
*)
	echo "ERROR: Unsupported agent '${AGENT}'. Expected claude or codex."
	exit 1
	;;
esac

REPO_ROOT="$(git -C "$(dirname "$(realpath "$0")")" rev-parse --show-toplevel)"
BASE_REF="origin/jazzy"
PROGRESS_FILE="progress.txt"

AGENT_COLORS=(blue cyan fuchsia green indigo lime magenta orange pink purple rose teal violet yellow)

# Find oldest open ready-for-agent issue without an open PR closing it (lowest number = oldest).
ISSUES_JSON=$(gh issue list \
	--label "ready-for-agent" \
	--state open \
	--limit 50 \
	--json number,title,body,closedByPullRequestsReferences |
	jq 'sort_by(.number)')

ISSUE_JSON=""
declare -A BLOCKED_BY_MAP # candidate number -> space-separated open blocker numbers
declare -A BLOCKS_MAP     # blocker number -> space-separated candidate numbers it blocks
SKIPPED_ISSUES=()

ROOT_BLOCKER=""

_print_dep_tree() {
	local skipped_set=" ${SKIPPED_ISSUES[*]} "

	local roots=()
	for blocker in "${!BLOCKS_MAP[@]}"; do
		if [[ "$skipped_set" != *" $blocker "* ]]; then
			roots+=("$blocker")
		fi
	done

	[[ ${#roots[@]} -eq 0 ]] && return 0

	mapfile -t roots < <(printf '%s\n' "${roots[@]}" | sort -n)
	ROOT_BLOCKER="${roots[0]}"

	echo ""
	echo "Dependency tree:"

	for root in "${roots[@]}"; do
		local root_title
		root_title=$(gh issue view "$root" --json title --jq '.title' 2>/dev/null || true)
		if [[ -n "$root_title" ]]; then
			printf '#%s — %s\n' "$root" "$root_title"
		else
			printf '#%s [external]\n' "$root"
		fi

		local children_str="${BLOCKS_MAP[$root]:-}"
		[[ -z "$children_str" ]] && continue

		local children=()
		read -ra children <<<"$children_str"
		mapfile -t children < <(printf '%s\n' "${children[@]}" | sort -n)

		local n=${#children[@]}
		local i=0
		for child in "${children[@]}"; do
			i=$((i + 1))
			local conn
			[[ $i -eq $n ]] && conn="└── " || conn="├── "

			local child_blockers="${BLOCKED_BY_MAP[$child]:-}"
			local other=()
			for b in $child_blockers; do
				[[ "$b" != "$root" ]] && other+=("#$b")
			done
			local also=""
			if [[ ${#other[@]} -gt 0 ]]; then
				also=" (also blocked by: $(
					IFS=', '
					echo "${other[*]}"
				))"
			fi

			local grandchildren="${BLOCKS_MAP[$child]:-}"
			local further=""
			if [[ -n "$grandchildren" ]]; then
				local gc_arr=()
				read -ra gc_arr <<<"$grandchildren"
				local gc_fmt
				gc_fmt=$(printf '#%s ' "${gc_arr[@]}")
				gc_fmt="${gc_fmt% }"
				further=" → blocks ${gc_fmt}"
			fi

			printf '%s#%s%s%s\n' "$conn" "$child" "$also" "$further"
		done
	done
	echo ""
}

_start_blocker_session() {
	local issue_data
	issue_data=$(gh issue view "$ROOT_BLOCKER" --json number,title,body 2>/dev/null) || return 0

	local issue_title issue_body
	issue_title=$(jq -r '.title' <<<"$issue_data")
	issue_body=$(jq -r '.body // ""' <<<"$issue_data")

	echo "Starting Claude session on root blocker #${ROOT_BLOCKER}: ${issue_title}"
	echo ""

	local prompt
	prompt="The following GitHub issue is blocking all ready-for-agent work in this repo.

Issue #${ROOT_BLOCKER}: ${issue_title}

${issue_body}

What should be done with this issue right now? Give me the most concrete next steps to make progress and unblock the queue."

	local color_idx
	color_idx=$(printf '%s' "$ROOT_BLOCKER" | cksum | awk '{print $1}')
	color_idx=$((color_idx % ${#AGENT_COLORS[@]}))

	claude \
		--name "blocker #${ROOT_BLOCKER} — ${issue_title}" \
		--agent-color "${AGENT_COLORS[$color_idx]}" \
		--permission-mode acceptEdits \
		"$prompt"
}

while IFS= read -r CANDIDATE_JSON; do
	CANDIDATE_NUMBER=$(echo "$CANDIDATE_JSON" | jq -r '.number')
	CANDIDATE_OPEN_PRS_JSON=$(
		while IFS= read -r CLOSING_PR_URL; do
			PR_JSON=$(gh pr view "$CLOSING_PR_URL" --json number,url,state)
			if [[ "$(jq -r '.state' <<<"$PR_JSON")" == "OPEN" ]]; then
				jq -c '{number, url, state}' <<<"$PR_JSON"
			fi
		done < <(jq -r '.closedByPullRequestsReferences[]?.url // empty' <<<"$CANDIDATE_JSON") |
			jq -s -c '.'
	)
	OPEN_PR_COUNT=$(echo "$CANDIDATE_OPEN_PRS_JSON" | jq 'length')

	if [[ "$OPEN_PR_COUNT" -gt 0 ]]; then
		OPEN_PRS=$(echo "$CANDIDATE_OPEN_PRS_JSON" |
			jq -r 'map(if .number then "#\(.number)" else .url end) | join(", ")')
		echo "Skipping issue #${CANDIDATE_NUMBER}; open PR already exists: ${OPEN_PRS}"
		continue
	fi

	# Skip issues whose "## Blocked by" section references a still-open issue.
	BLOCKER_NUMBERS=$(jq -r '.body // ""' <<<"$CANDIDATE_JSON" |
		awk '/^## Blocked by/{flag=1; next} /^## /{flag=0} flag && /^- /' |
		grep -oE '#[0-9]+' | tr -d '#' | sort -un || true)
	OPEN_BLOCKERS=""
	OPEN_BLOCKER_NUMS=()
	for BLOCKER in $BLOCKER_NUMBERS; do
		BLOCKER_STATE=$(gh issue view "$BLOCKER" --json state --jq '.state' 2>/dev/null || echo "UNKNOWN")
		if [[ "$BLOCKER_STATE" == "OPEN" ]]; then
			OPEN_BLOCKERS="${OPEN_BLOCKERS} #${BLOCKER}"
			OPEN_BLOCKER_NUMS+=("$BLOCKER")
			BLOCKS_MAP[$BLOCKER]="${BLOCKS_MAP[$BLOCKER]:+${BLOCKS_MAP[$BLOCKER]} }${CANDIDATE_NUMBER}"
		fi
	done
	if [[ -n "$OPEN_BLOCKERS" ]]; then
		echo "Skipping issue #${CANDIDATE_NUMBER}; blocked by open issue(s):${OPEN_BLOCKERS}"
		BLOCKED_BY_MAP[$CANDIDATE_NUMBER]="${OPEN_BLOCKER_NUMS[*]}"
		SKIPPED_ISSUES+=("$CANDIDATE_NUMBER")
		continue
	fi

	ISSUE_JSON="$CANDIDATE_JSON"
	break
done < <(echo "$ISSUES_JSON" | jq -c '.[]')

ISSUE_NUMBER=$(echo "$ISSUE_JSON" | jq -r '.number')
ISSUE_TITLE=$(echo "$ISSUE_JSON" | jq -r '.title')

if [[ -z "$ISSUE_NUMBER" || "$ISSUE_NUMBER" == "null" ]]; then
	echo "No open ready-for-agent issues without open PRs found."

	if [[ ${#SKIPPED_ISSUES[@]} -gt 0 ]]; then
		_print_dep_tree
		if [[ -n "$ROOT_BLOCKER" ]]; then
			_start_blocker_session
			exit 0
		fi
	fi

	exit 1
fi

# Slugify: strip [Project] prefix, lowercase, non-alnum → hyphen, trim to 40 chars
SLUG=$(echo "$ISSUE_TITLE" |
	sed 's/^\[.*\] *//' |
	tr '[:upper:]' '[:lower:]' |
	sed 's/[^a-z0-9]/-/g' |
	sed 's/-\+/-/g' |
	sed 's/^-\|-$//g' |
	cut -c1-40 |
	sed 's/-$//g')

BRANCH="feature/${ISSUE_NUMBER}-${SLUG}"
WORKTREE_DIR="${REPO_ROOT}/.worktrees/${ISSUE_NUMBER}-${SLUG}"

echo "Issue #${ISSUE_NUMBER}: ${ISSUE_TITLE}"
echo "Branch:   ${BRANCH}"
echo "Worktree: ${WORKTREE_DIR}"
echo "Agent:    ${AGENT}"

# Refresh remote references before selecting the feature branch's baseline.
git -C "$REPO_ROOT" fetch origin

# Remove worktrees whose closing PR has been merged.
cleanup_merged_worktrees() {
	local repo_root="$1"
	local worktrees_base="${repo_root}/.worktrees"

	[[ -d "$worktrees_base" ]] || return 0

	for worktree_dir in "${worktrees_base}"/*/; do
		[[ -d "$worktree_dir" ]] || continue
		local dir_name issue_num is_merged pr_url state
		dir_name=$(basename "$worktree_dir")
		issue_num="${dir_name%%-*}"
		[[ "$issue_num" =~ ^[0-9]+$ ]] || continue

		is_merged=false
		while IFS= read -r pr_url; do
			[[ -z "$pr_url" ]] && continue
			state=$(gh pr view "$pr_url" --json state --jq '.state' 2>/dev/null || true)
			if [[ "$state" == "MERGED" ]]; then
				is_merged=true
				break
			fi
		done < <(gh issue view "$issue_num" --json closedByPullRequestsReferences \
			--jq '.closedByPullRequestsReferences[].url' 2>/dev/null || true)

		if [[ "$is_merged" == "true" ]]; then
			echo "Removing merged worktree: ${dir_name} (issue #${issue_num})"
			git -C "$repo_root" worktree remove --force "$worktree_dir" 2>/dev/null || rm -rf "$worktree_dir"
		fi
	done
}

cleanup_merged_worktrees "$REPO_ROOT"

# Remove stale worktree registrations whose directories were manually deleted.
git -C "$REPO_ROOT" worktree prune

# Create worktree (or reuse if already exists)
if [[ -d "$WORKTREE_DIR" ]]; then
	echo "Reusing existing worktree"
elif git -C "$REPO_ROOT" show-ref --verify --quiet "refs/heads/${BRANCH}" 2>/dev/null; then
	# Branch already exists (interrupted previous run) — attach worktree to it
	git -C "$REPO_ROOT" worktree add "$WORKTREE_DIR" "$BRANCH"
	echo "Created worktree on existing branch"
else
	git -C "$REPO_ROOT" worktree add --no-track "$WORKTREE_DIR" -b "$BRANCH" "$BASE_REF"
	echo "Created worktree on new branch"
fi

cd "$WORKTREE_DIR"

if [[ -n "$(git status --porcelain)" ]]; then
	echo "WARNING: Worktree has pending changes; skipping automatic branch synchronization"
else
	if git show-ref --verify --quiet "refs/remotes/origin/${BRANCH}" 2>/dev/null; then
		if ! git merge --ff-only "origin/${BRANCH}"; then
			echo "ERROR: Local and remote feature branches have diverged; resolve them before rerunning"
			exit 1
		fi
	fi

	if ! git merge-base --is-ancestor "$BASE_REF" HEAD; then
		if git merge --no-edit "$BASE_REF"; then
			echo "Merged latest ${BASE_REF} into ${BRANCH}"
		else
			git merge --abort
			echo "ERROR: ${BRANCH} conflicts with ${BASE_REF}; merge it manually before rerunning"
			exit 1
		fi
	fi
fi

PROMPT="@${PROGRESS_FILE}
1. Read GitHub issue #${ISSUE_NUMBER} and the progress file.
2. Implement issue #${ISSUE_NUMBER}.
3. Commit your changes.
4. Update ${PROGRESS_FILE} with what you did.
5. Push the branch and open a PR.
6. The PR description must include 'Closes #${ISSUE_NUMBER}'.
ONLY DO ONE TASK AT A TIME."

COLOR_INDEX=$(printf '%s' "$ISSUE_NUMBER" | cksum | awk '{print $1}')
COLOR_INDEX=$((COLOR_INDEX % ${#AGENT_COLORS[@]}))
AGENT_COLOR="${AGENT_COLORS[$COLOR_INDEX]}"

case "$AGENT" in
claude)
	claude --name "#${ISSUE_NUMBER} - ${ISSUE_TITLE}" --agent-color "$AGENT_COLOR" --permission-mode acceptEdits "$PROMPT"
	;;
codex)
	codex "$PROMPT"
	;;
esac
