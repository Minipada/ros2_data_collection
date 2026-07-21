# DC (ROS 2 Data Collection) — Claude Context

DC collects operational data from robots (system, robot-state, and inspection measurements),
validates it, and routes it to external infrastructure (databases, object storage, APIs) that
powers analytics and dashboards. It is a telemetry pipeline, not an ML dataset platform.

Read [CONTEXT.md](./CONTEXT.md) first for the domain vocabulary (Measurement, Record, Group,
Destination, Bridge, Shipper, Tag, File, …) — use those terms, not synonyms, in code, commits,
and PRs. Read [docs/adr/](./docs/adr/) for the DC 2.0 architecture decisions (external Vector
shipper replacing embedded Fluent Bit, Rust bridge, blessed destinations + passthrough, etc.)
before touching pipeline internals — most non-obvious design choices are already recorded there
rather than in code comments.

## Branches

- `humble` — legacy/stable line, ROS 2 Humble, embedded Fluent Bit architecture. CI
  (`.github/workflows/ci.yaml`) still targets this branch.
- `jazzy` — active development target for the DC 2.0 rewrite (external Vector shipper, Rust
  bridge, ROS 2 Jazzy). New work — including everything `run_once.sh` picks up — branches from
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

CI (`ci.yaml`) builds/tests through `industrial_ci` inside Docker per ROS distro — there's no
single local `colcon test` invocation that reproduces it exactly; `pre-commit` plus a local
`colcon build` covers what an agent can practically verify before opening a PR.

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
