# DC (ROS 2 Data Collection) — Claude Context

DC collects operational data from robots (system, robot-state, and inspection measurements),
validates it, and routes it to external infrastructure (databases, object storage, APIs) that
powers analytics and dashboards. It is a telemetry pipeline, not an ML dataset platform.

Read [CONTEXT.md](./CONTEXT.md) first for the domain vocabulary (Measurement, Record, Group,
Destination, Bridge, Shipper, Tag, File, …) — use those terms, not synonyms, in code, commits,
and PRs. Read [docs/adr/](./docs/adr/) for the DC 2.0 architecture decisions (external Vector
shipper replacing embedded Fluent Bit, the C++ Bridge, blessed destinations +
passthrough, etc.) before touching pipeline internals — most non-obvious design choices
are already recorded there rather than in code comments.

## Branches

- `humble` — legacy/stable line, ROS 2 Humble, embedded Fluent Bit architecture. CI
  (`.github/workflows/ci.yaml`) still targets this branch.
- `jazzy` — active development target for the DC 2.0 rewrite (external Vector shipper, C++
  Bridge, ROS 2 Jazzy). New work — including everything `run_once.sh` picks up — branches from
  and merges into `jazzy`, not `humble`.

## Repo layout

| Package | Purpose |
|---|---|
| `dc_bringup` | Bringup scripts and launch configuration for the DC stack |
| `dc_core` | Headers for plugins core to the DC stack |
| `dc_common` | Common support functionality used throughout the DC stack |
| `dc_measurements` | Collect data with Measurement plugins |
| `dc_triggers` | Trigger plugins and the broadcast node publishing FlushEvents |
| `dc_group` | Group node — merges Records from several Measurements by time proximity |
| `dc_services` | Collect uptime data |
| `dc_lifecycle_manager` | Controller/manager for the DC system's lifecycle nodes |
| `dc_interfaces` | Data collection ROS interfaces (msgs/srvs) |
| `dc_cli` | DC CLI tools |
| `dc_description` | Description files for DC |
| `dc_demos` | Demo packages |
| `dc_simulation` | Warehouse simulation |
| `dc_util` | General shared headers |

## Build / lint

```bash
colcon build                 # requires a sourced ROS 2 workspace; see doc/src/dc/setup.md
uv sync                      # Python deps: pyproject.toml + uv.lock, no requirements.txt
git add -A && prek run --all-files --skip build-doc   # see .pre-commit-config.yaml
```

`--all-files` means all *tracked* files: an unstaged new file is skipped silently, so a
formatting break in it only surfaces in CI (#359). Stage first, hence the `git add -A`.

Python tooling is **prek + uv**, not pre-commit + poetry (same move as `~/dev/monorepo`),
and lint/format is **ruff only** (`[tool.ruff]` in `pyproject.toml`). There are no
`requirements*.txt` any more — `pyproject.toml` + `uv.lock` are the dependency source.
Skip `build-doc` outside a docs change: it builds the docs container, which `doc.yaml`
owns in CI.

CI (`.github/workflows/ci.yaml`) builds the DC workspace with Podman and runs `colcon
test` (C++ gtest across every package) against it — see "Containers: Podman, not
Docker" below. Its `build-workspace` job runs the exact same `tools/e2e/scripts/build.sh`
a developer runs locally, so CI and local dev build `tools/e2e/Containerfile` the same
way; `colcon test` is part of that same build (the Containerfile's `workspace` stage),
so CI no longer calls `test.sh` directly. CI adds two things local dev doesn't need:
`CACHE_REF`, a registry ref `build.sh` passes to `podman build --cache-from`/
`--cache-to` so the rarely-changing apt/toolchain/aws_sdk_vendor *layers* stay warm
on a cold GitHub-hosted runner (podman's layer cache is otherwise local to one
machine); and `BUILDAH_TMPDIR`, which relocates the `workspace` stage's ccache
`RUN --mount=type=cache` mount into a directory `actions/cache` persists between
runs — that mount lives under buildah's own `$TMPDIR`, a mechanism
`--cache-from`/`--cache-to` does not touch at all (verified against two live CI runs:
the mount showed a 0% hit rate under `--cache-to`/`--cache-from` alone). Together
they mean a change to one source file only recompiles that file's translation units,
not the whole workspace, on a cold runner too — no bind mounts, no separate
incremental-build script. Note: `ci.yaml` on the `humble` branch is a *different*
file (that branch's own tree, `industrial_ci` inside Docker) — same filename,
unrelated content, since each
branch keeps its own `.github/workflows/`.

## Containers: Podman, not Docker

New container tooling in this repo (CI images, E2E harness) uses **Podman**, not
Docker — Docker is being phased out repo-wide, matching the direction already taken in
`~/dev/monorepo` (see `~/dev/monorepo/server/docs/adr/002-rootless-podman-quadlet.md`
and `~/dev/monorepo/ci/Containerfile` + `.github/actions/{image-ref,docker-build,
podman-push,registry-login,trivy-image-scan}` for the reference conventions this repo
follows). Most of the pre-existing `docker/` tree, root `docker-compose.yaml`, and
`.github/workflows/docker.yaml` were humble-line legacy — removed from this branch's
tree (they still exist on `humble`'s own tree, a separate branch/tree; deleting them
here doesn't touch that): `.github/workflows/ci.yaml` on this branch was replaced
outright by the jazzy-line Podman CI, and the industrial_ci `ci`/`ci-testing`/`source`/
`source-sim` jobs those files served were unreachable anyway on a branch that can't
build `fluent_bit_plugins`/`dc_destinations` in the first place — both `COLCON_IGNORE`d
per ADR-0001/#242. `docker/doc/Dockerfile` (the mdbook docs-site builder) was renamed to
match the Podman convention below (`containers/doc/Containerfile`) but is **not yet
live** — the `docker.yaml` job that built and pushed its image is gone along with the
rest, and nothing has replaced it; `.github/workflows/doc.yaml` and
`tools/ci/pre-commit/build_doc.sh` still point at the now-stale published tag. Wiring a
real jazzy docs build is tracked at #252 (DC 2.0 S11), not done as part of this cleanup.
New Podman-based container tooling otherwise lives under `tools/e2e/` (#249 is the first
jazzy-line CI/container work).

**Scoped exception — k3d/kind:** `deploy/robot/scripts/k3d_up.sh`/`k3d_down.sh` (#451, a
fast local dev loop for the Kubernetes rendering) and #452's kind + NetworkPolicy
validation both wrap Kubernetes distributions that run their nodes as Docker containers —
neither ships a Podman-based option. The dependency is confined to those two harnesses:
building and shipping `dc-ros`/`dc-uploader` still goes through Podman unchanged, nothing
else in the repo assumes Docker is present, and neither harness runs in CI.

Conventions for new container tooling here, adapted from the monorepo (no private
registry or self-hosted runners in this repo, so the parts of that pattern needing
those are dropped):

- **`Containerfile`, not `Dockerfile`** as the filename (Podman/OCI convention).
- **Fully-qualified base images** (`docker.io/library/ros:...`, not bare `ros:...`) —
  Podman has no default unqualified-search registry configured, unlike Docker.
- **`podman build`**/**`podman run`**, not `docker build`/`docker run`. Use
  `podman compose` (not `docker compose`/`docker-compose`) only for stores that stay up
  for a whole run with no lifecycle control needed; drive containers directly with
  `podman run`/`stop`/`restart` when the tooling itself needs to control their lifecycle
  (e.g. the zero-loss E2E harness's induced outage + restart, `tools/e2e/scripts/run.sh`
  — compose has no supported way to stop/restart one of its services mid-run).
- **Tag images with the commit SHA** (`<name>:<github.sha>`), mirroring the monorepo's
  `image-ref` action's immutable `:<sha>` ref — even for images that stay local to a CI
  run rather than being pushed to a registry, so a build is traceable back to the
  commit that produced it. Add a floating branch/tag ref alongside it only once this
  repo actually publishes images for others to pull and run (it doesn't yet — no
  registry is configured for that here the way `registry.bensoussan.de` is in the
  monorepo). `ci.yaml` does push to `ghcr.io` today, but only as a `podman build
  --cache-to`/`--cache-from` **build cache** (verified working — see
  `tools/e2e/scripts/build.sh`'s `CACHE_REF`), which is a different thing from
  publishing a runnable image.
- **`hadolint`** lints every Containerfile (already wired into `.pre-commit-config.yaml`
  for the legacy `docker/*/Dockerfile`s — new `Containerfile`s are covered by the same
  hook via its glob).

## Comments

Keep them short. A file header is one to three lines; a block comment is one or two.
Say *why*, and only when the code can't. No history lessons, no restating the diff, no
paragraphs of rationale — that belongs in an ADR or the PR description.
Some older files (`ci.yaml`, `doc.yaml`, `tools/e2e/Containerfile`) have long headers
from before this rule; don't copy them, and trim them when you touch them.

## Issue-tracker workflow (used by `run_once.sh`)

- Issues ready for autonomous agent work carry the `ready-for-agent` label.
- An issue can declare dependencies with a `## Blocked by` section listing `#N` issue numbers;
  agents must not start an issue while any listed blocker is still open.
- Read the target GitHub issue before implementing; git log/PR history is the record of
  completed work — there is no separate progress log to maintain.
- One task per run/session. Commit, push the branch, and open a PR whose description includes
  `Closes #N`.
- `run_once.sh` automates all of the above: it picks the oldest eligible issue, does the work in
  an isolated `git worktree` under `.worktrees/`, and hands the prompt to `claude` or `codex`.
  `--issue N` (or a bare `N`, or `RUN_ONCE_ISSUE=N`) works on that issue instead — the label,
  open-PR and `Blocked by` filters then only warn, since they exist to pick a candidate.
