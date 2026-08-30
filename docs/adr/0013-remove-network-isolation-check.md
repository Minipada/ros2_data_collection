# Remove the network-isolation-check CI job and vendor-network-check stage

`tools/e2e/Containerfile`'s `vendor-network-check` stage and CI's `network-isolation-check`
job (#423) are deleted, along with `tools/e2e/scripts/verify_network_isolation.sh`. The
check they implemented — "does `aws_sdk_vendor`'s and `vector_vendor`'s `colcon build`
succeed under `--network=none`" — no longer has anything to verify: both packages fetch
their dependency live over the network at `colcon build` time by design (docs/adr/0002's
reversal, #435; docs/adr/0012, #434), so the check now fails by construction, always, for
both packages it covers.

## Why

The check was built (#423) on the assumption that ROS buildfarm binarydeb jobs run with no
network access. #434's research (`ros_buildfarm`'s own `docker run --net=host` for the
binarydeb build step, `zmqpp_vendor`'s real succeeding buildfarm job using an equivalent
live-fetch pattern) showed that assumption false. Once #434 and #435 reversed both vendor
packages back to live network fetches, the check stopped testing anything achievable: it
fails on every run, forever, for the same reason every time, and nothing about that reason
is a bug it could ever stop reproducing — it's the intended design those two ADRs record.

The job had already been reduced to `if: false` before this decision, because a disabled
job still reports a red ❌ check-run conclusion on every PR (independent of
`continue-on-error`, which only affects the *workflow's* aggregate conclusion), with
nothing actionable behind it. Keeping the job, the stage, and the script around in that
disabled state is dead weight: code that never runs, whose only content is restating a
decision already recorded in docs/adr/0002 and docs/adr/0012.

Two alternative rationales were considered for keeping some form of the check, both raised
by #436 itself:

- **Build-determinism/reproducibility safeguard.** A check earning that rationale would
  assert something about the *build output's* reproducibility — that the pinned checksum
  (`vector_vendor`) or pinned tag (`aws_sdk_vendor`) actually resolves to the same content
  run over run. `--network=none` asserts nothing of the kind; it only asserts "no network
  reached this step," which is now always false by design. Repurposing the stage into a
  real reproducibility check would mean writing a materially different one, not weakening
  the existing one, and no evidence surfaced during #434/#435 that reproducibility of the
  pinned fetches is a risk this repo has actually hit.
- **Protection against transient upstream (GitHub/crates/apt mirror) unavailability
  during a real release build.** A genuine operational concern, but `--network=none`
  doesn't model it — cutting network entirely is the opposite of a flaky network, not a
  simulation of one. A release build failing because GitHub is briefly unreachable is
  already visible, attributable, and retryable the same way any other CI network flake is,
  with no dedicated isolation check required. Retry-on-flake, if wanted later, is a
  `bloom-release`/CI workflow concern (retries, backoff) — unrelated to build-time network
  isolation.

Neither rationale survives contact with what the existing check actually mechanically
tests, so there is no "weakened" version of it worth keeping.

## Decision

- `tools/e2e/Containerfile`'s `vendor-network-check` stage is deleted.
- `.github/workflows/ci.yaml`'s `network-isolation-check` job is deleted.
- `tools/e2e/scripts/verify_network_isolation.sh` is deleted.
- `toolchain-base` — the stage `vendor-network-check` used to branch from — is kept: it
  still earns its keep as the cache-layering split between the OS/apt-tools layer and the
  vendor-repo `vcs import`, independent of the now-removed stage that used to branch from
  it too.
- docs/adr/0002 and docs/adr/0012's own descriptions of the check as it existed at the
  time each ADR was written are left as historical record; each links here where it
  implied the check would keep running going forward.

## Consequences

- No CI job or Containerfile stage references "network isolation" for these vendor
  packages any more — that they fetch live, by design, is fully covered by docs/adr/0002
  and docs/adr/0012 alone.
- If a future ROS buildfarm policy change restricts binarydeb network access again (the
  false premise both those ADRs found reversing back to true), reintroducing a real
  network-isolated build check is straightforward: `RUN --network=none` around a
  `colcon build --packages-select aws_sdk_vendor vector_vendor` is exactly what this ADR
  removes, and both ADRs it depends on document why it existed and how it worked.
- #423 itself stays closed with its history unchanged — this ADR documents its follow-up
  removal, not a re-litigation of the original harness's value at the time it was built.
