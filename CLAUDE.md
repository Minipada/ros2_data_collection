# DC (ROS 2 Data Collection) — Claude Context

DC collects operational data from robots (system, robot-state, and inspection measurements),
validates it, and routes it to external infrastructure (databases, object storage, APIs) that
powers analytics and dashboards. It is a telemetry pipeline, not an ML dataset platform.

Read [CONTEXT.md](./CONTEXT.md) first for the domain vocabulary (Measurement, Record, Group,
Destination, Bridge, Shipper, Tag, File, …) — use those terms, not synonyms, in code, commits,
and PRs. Read [docs/adr/](./docs/adr/) for the DC 2.0 architecture decisions (external Vector
shipper replacing embedded Fluent Bit, the C++ Bridge — ADR-0007 reverted the ADR-0004 Rust
pilot, blessed destinations + passthrough, etc.) before touching pipeline internals — most
non-obvious design choices are already recorded there rather than in code comments.

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
| `dc_group` | Group node — merges Records from several Measurements by time proximity |
| `dc_destinations` | Send Records to Destination plugins |
| `dc_services` | Collect uptime data |
| `dc_lifecycle_manager` | Controller/manager for the DC system's lifecycle nodes |
| `dc_interfaces` | Data collection ROS interfaces (msgs/srvs) |
| `dc_cli` | DC CLI tools |
| `dc_description` | Description files for DC |
| `dc_demos` | Demo packages |
| `dc_simulation` | Warehouse simulation |
| `dc_util` | General shared headers |
| `fluent_bit_plugins`, `fluent_bit_vendor` | Embedded Fluent Bit (humble-era); being demolished per ADR-0001 in the `jazzy` line |

## Build / lint

```bash
colcon build                 # requires a sourced ROS 2 workspace; see doc/src/dc/setup.md
pip3 install -r requirements.txt   # or `poetry install`
pre-commit run --all-files   # flake8, doc build, yaml/json/xml checks, etc. — see .pre-commit-config.yaml
```

CI (`.github/workflows/ci.yaml`) builds the shared DC workspace image
(`tools/e2e/Containerfile`) with Podman and runs `colcon test` (C++ gtest across every
package) against it — see "Containers: Podman, not Docker" below. `tools/e2e/scripts/build.sh` /
`test.sh` are the same scripts CI calls, runnable locally too. Note: `ci.yaml` on the
`humble` branch is a *different* file (that branch's own tree, `industrial_ci` inside
Docker) — same filename, unrelated content, since each branch keeps its own
`.github/workflows/`.

## Containers: Podman, not Docker

New container tooling in this repo (CI images, E2E harness) uses **Podman**, not
Docker — Docker is being phased out repo-wide, matching the direction already taken in
`~/dev/monorepo` (see `~/dev/monorepo/server/docs/adr/002-rootless-podman-quadlet.md`
and `~/dev/monorepo/ci/Containerfile` + `.github/actions/{image-ref,docker-build,
podman-push,registry-login,trivy-image-scan}` for the reference conventions this repo
follows). The pre-existing `docker/` tree, root `docker-compose.yaml`, and
`.github/workflows/docker.yaml` are humble-line legacy — they exist only on the
`humble` branch's own tree at this point (`.github/workflows/ci.yaml` on this branch
was replaced outright by the jazzy-line Podman CI; there was no reason to keep an
unreachable Docker/industrial_ci workflow around on a branch that can't build
`fluent_bit_plugins`/`dc_destinations` in the first place — both `COLCON_IGNORE`d per
ADR-0001/#242). New Podman-based container tooling lives under `tools/e2e/` (#249 is
the first jazzy-line CI/container work).

Conventions for new container tooling here, adapted from the monorepo (no private
registry or self-hosted runners in this repo, so the parts of that pattern needing
those are dropped):

- **`Containerfile`, not `Dockerfile`** as the filename (Podman/OCI convention).
- **Fully-qualified base images** (`docker.io/library/ros:...`, not bare `ros:...`) —
  Podman has no default unqualified-search registry configured, unlike Docker.
- **`podman build`**/**`podman run`**/**`podman compose`**, not `docker build`/`docker
  run`/`docker compose` or `docker-compose`.
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

## Issue-tracker workflow (used by `run_once.sh`)

- Issues ready for autonomous agent work carry the `ready-for-agent` label.
- An issue can declare dependencies with a `## Blocked by` section listing `#N` issue numbers;
  agents must not start an issue while any listed blocker is still open.
- `progress.txt` at repo root is the running log of completed work — read it and the target
  GitHub issue before implementing, update it with what you did, and commit it alongside the
  code change (it's tracked in git, not gitignored — the log is part of the durable state a
  fresh agent session reconstructs context from).
- One task per run/session. Commit, push the branch, and open a PR whose description includes
  `Closes #N`.
- `run_once.sh` automates all of the above: it picks the oldest eligible issue, does the work in
  an isolated `git worktree` under `.worktrees/`, and hands the prompt to `claude` or `codex`.
