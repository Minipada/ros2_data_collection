#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Drain-rate axis (#385, part of #323's limits harness): "how long does a backlog take
to clear once connectivity returns, as a function of load and outage length."

An **outage cycle** drives synthetic load (tools/e2e/scripts/load_driver.py, #378) at a
stated connection count and per-connection rate against a Shipper under test while its
downstream Destination is unreachable — reusing tools/e2e/scripts/run.sh's own
outage-inducing recipe (`podman stop`, wait, `podman start`) unmodified rather than
inventing a second way to simulate an outage — then, once the Destination is restored,
polls the Shipper's buffer size until it clears (or a wait bound expires) and reports
that elapsed time as the drain time.

**Buffer size comes from Vector's own `internal_metrics` + `prometheus_exporter`
(`vector_buffer_byte_size{buffer_id=...}`), not a filesystem stat on the buffer
directory.** Verified empirically while building this module: Vector's on-disk buffer
format (a segment-based mmap log) does not shrink its `.dat` segment file as events are
acked — `du -sb` on the buffer directory stayed pinned at its post-outage peak for over a
minute of continuous real deliveries in a live test run here, which would have made
every drain look like it never happened. The `vector_buffer_byte_size` gauge, by
contrast, tracks the buffer's actual pending bytes and dropped to 0 within ~8s of the
Destination coming back in the same test. This is exactly the mechanism #323's PRD
anticipates ("the harness injects a metrics source and exporter into either tier for the
duration of a run" — "Observability comes from configuration, not code changes"):
run_limits_drain_rate.sh's own Vector config carries no dc_bridge/DC involvement to
begin with, so adding `internal_metrics`/`prometheus_exporter` to it is not a DC code
change at all.

This module splits cleanly along the epic's own line between "deep" (pure, no I/O,
unit-tested with fakes) and "integration" (real subprocess/podman, exercised by actually
running it) modules:

- `evaluate_cycle()` / `find_drain_rate_curve()` are pure functions over
  `DrainCycleResult` data and are unit-tested the same way ramp_controller's find_knee()
  is: with a fake driver, no containers, no network (test_drain_rate_axis.py).
- `make_real_cycle_driver()` / `main()` do the actual podman/subprocess/load_driver.py
  work and are exercised by tools/e2e/scripts/run_limits_drain_rate.sh, not unit-tested,
  matching the epic's "the topology composer and the scenario script are not
  unit-tested; running them is the test" testing decision.

**The axis reuses saturation_probe.evaluate() itself for the outage-aware signal rather
than a bespoke check** (#385's own acceptance criterion): every cycle's outage-phase
buffer-size samples are run through the exact same disk-buffer-growth signal #377
defines, with `outage_declared=True`, and the cycle raises `OutageMisreportedError` if
that ever trips — the growth is real and expected here, so a trip would mean the
outage-aware suppression in saturation_probe.py itself is broken, not that this axis
found a legitimate result. The recovery-phase samples reuse the identical signal with
`outage_declared=False`: a backlog that is still growing (or has stopped shrinking)
after connectivity returns is exactly what that signal is built to catch, so no second
implementation of "is the buffer trending up" exists in this module.

ack_latency_s and unacked_window_depth are fixed at 0.0 for every Observation this
module builds — deliberately inert (always below any positive bound), because this
axis's job is to exercise the disk-buffer signal specifically; the other two signals are
the fan-in/single-robot-ceiling axes' concern.

**Ramping.** At a fixed stated load (a script parameter — callers vary it across
invocations, the same convention run_limits_two_tier.sh uses for REAL_STACKS /
SYNTH_CONNECTIONS rather than sweeping them in one run), `find_drain_rate_curve()` reuses
ramp_controller.find_knee() to drive an ascending list of outage lengths and reports the
knee: the shortest outage length whose backlog failed to drain within the wait bound
(never fabricated — a policy whose every outage length drains cleanly is reported as
`RunOutcome.BOUND_NOT_FOUND`, matching #379's own "never fabricate a ceiling" rule).
`AxisRun.points[].level` is therefore the ramped **outage length in seconds**, matching
the unit `tripped_level`/`highest_clear_level` are reported in (they come straight out of
`RampResult`, which is defined in the policy's own unit) — the per-level *drain time*
this axis exists to report is a different number and doesn't fit any existing
curve_reporter field, so it is written alongside the curve report as a companion JSON
(see `main()`), keyed by the same outage-length levels the report's points use.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.request
from collections.abc import Callable, Sequence
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from curve_reporter import (
    AxisRun,
    LevelResult,
    PointKind,
    RunComposition,
    RunOutcome,
    build_report,
    render_summary,
    to_json,
)
from ramp_controller import RampOutcome, RampPolicy, RampResult, find_knee
from saturation_probe import (
    Observation,
    SaturationBounds,
    Verdict,
    VerdictStatus,
    evaluate,
)


class OutageMisreportedError(RuntimeError):
    """Raised when a cycle's outage-phase window trips SATURATED (or can't be assessed
    at all) — a correctness bug in the outage-aware suppression itself, never an ordinary
    ramp result. find_knee() must never see this as "level N saturated"; the whole run
    aborts instead of silently reporting a knee that isn't really one."""


@dataclass(frozen=True)
class DrainCycleResult:
    """One induced-outage-then-recovery cycle at a fixed load. `drain_time_s` is `None`
    exactly when the backlog never cleared within the wait bound — never coerced to the
    bound itself, since "still draining slowly" and "never came close" are different
    findings for whoever reads the report."""

    outage_length_s: float
    connections: int
    rate_hz: float
    peak_buffer_bytes: float
    drain_time_s: float | None
    outage_window: tuple[Observation, ...]
    recovery_window: tuple[Observation, ...]


@dataclass(frozen=True)
class CycleVerdict:
    """The `SaturationVerdict` ramp_controller.find_knee() needs (a `.saturated`
    property), plus both probe evaluations that produced it, for the caller to log or
    persist."""

    outage_verdict: Verdict
    recovery_verdict: Verdict | None
    saturated: bool
    reason: str


def evaluate_cycle(
    cycle: DrainCycleResult, bounds: SaturationBounds = SaturationBounds()
) -> CycleVerdict:
    """Pure function, no I/O: the module's own acceptance-criterion proof that the
    outage-aware signal is exercised, not reimplemented. Raises `OutageMisreportedError`
    if the outage-phase window is ever reported saturated (or can't be assessed) despite
    `outage_declared=True` throughout it — a real growth in that window is expected and
    must never trip. Otherwise the *recovery*-phase window (`outage_declared=False`) is
    evaluated with the identical signal, and its verdict decides whether this outage
    length is the ramp's knee — except when the backlog never drained at all
    (`drain_time_s is None`): that is always reported saturated, since a buffer that has
    merely plateaued rather than grown would not otherwise trip the growth-trend check,
    and "never fabricate a clear result" is #379's own rule for this ramp too.
    """
    outage_verdict = evaluate(cycle.outage_window, bounds)
    if outage_verdict.status == VerdictStatus.INSUFFICIENT_DATA:
        raise OutageMisreportedError(
            f"only {len(cycle.outage_window)} buffer sample(s) were captured during the "
            f"{cycle.outage_length_s}s outage — too few to exercise the outage-aware "
            "signal at all; shorten the poll interval or lengthen the outage"
        )
    if outage_verdict.saturated:
        raise OutageMisreportedError(
            f"buffer growth during the declared {cycle.outage_length_s}s outage was "
            f"reported as saturation ({outage_verdict.reason}) — the outage-aware "
            "suppression in saturation_probe.py is broken"
        )

    if cycle.drain_time_s is None:
        return CycleVerdict(
            outage_verdict=outage_verdict,
            recovery_verdict=None,
            saturated=True,
            reason=(
                f"backlog did not drain within the wait bound after the "
                f"{cycle.outage_length_s}s outage ended"
            ),
        )

    recovery_verdict = evaluate(cycle.recovery_window, bounds)
    return CycleVerdict(
        outage_verdict=outage_verdict,
        recovery_verdict=recovery_verdict,
        saturated=recovery_verdict.saturated,
        reason=recovery_verdict.reason,
    )


def build_axis_run(
    ramp_result: RampResult,
    cycles_by_level: dict[float, DrainCycleResult],
    connections: int,
    rate_hz: float,
) -> AxisRun:
    """Pure function: turns one find_knee() run plus the cycles it drove into an
    `AxisRun` in curve_reporter's own shape (#385's "same stable report format as the
    other axes" acceptance criterion) — no new fields, no schema change. `level` is the
    outage length in seconds tried at that point (see the module docstring for why the
    per-level *drain time* travels in a companion JSON instead)."""
    points = [
        LevelResult(
            level=level,
            saturated=(level == ramp_result.tripped_level),
            kind=PointKind.MEASURED,
            composition=RunComposition(real_stacks=0, synthetic_senders=connections),
        )
        for level in ramp_result.levels_tried
    ]
    outcome = (
        RunOutcome.KNEE_FOUND
        if ramp_result.outcome == RampOutcome.KNEE_FOUND
        else RunOutcome.BOUND_NOT_FOUND
    )
    return AxisRun(
        axis="drain_rate",
        unit=f"seconds (outage length @ {connections} synthetic connection(s) x {rate_hz}Hz)",
        outcome=outcome,
        highest_clear_level=ramp_result.highest_clear_level,
        tripped_level=ramp_result.tripped_level,
        points=points,
    )


def find_drain_rate_curve(
    driver: Callable[[float], DrainCycleResult],
    outage_lengths_s: Sequence[float],
    connections: int,
    rate_hz: float,
    bounds: SaturationBounds = SaturationBounds(),
) -> tuple[AxisRun, dict[float, DrainCycleResult]]:
    """Wires `driver` (real or fake) into ramp_controller.find_knee() via
    `evaluate_cycle()`, ascending through `outage_lengths_s`, and returns both the
    resulting `AxisRun` and the raw per-level cycle data (for the companion drain-time
    JSON `main()` writes). No I/O of its own — `driver` supplies all of it — so this is
    the seam test_drain_rate_axis.py exercises with a fake driver, the same pattern
    test_ramp_controller.py uses for find_knee() itself."""
    cycles_by_level: dict[float, DrainCycleResult] = {}

    def wrapped_driver(level: float) -> DrainCycleResult:
        cycle = driver(level)
        cycles_by_level[level] = cycle
        return cycle

    def probe(cycle: DrainCycleResult) -> CycleVerdict:
        return evaluate_cycle(cycle, bounds)

    ramp_result = find_knee(wrapped_driver, probe, RampPolicy(levels=outage_lengths_s))
    axis_run = build_axis_run(ramp_result, cycles_by_level, connections, rate_hz)
    return axis_run, cycles_by_level


# --- integration: the real driver (subprocess/podman), not unit-tested -------------------


def _podman(*args: str) -> None:
    subprocess.run(["podman", *args], check=True, stdout=subprocess.DEVNULL)


_BUFFER_BYTE_SIZE_RE = re.compile(
    r'^vector_buffer_byte_size\{[^}]*buffer_id="(?P<buffer_id>[^"]+)"[^}]*\}\s+(?P<value>[0-9.eE+-]+)',
    re.MULTILINE,
)


def _buffer_bytes(metrics_url: str, buffer_id: str) -> float:
    """Scrapes Vector's own `vector_buffer_byte_size` gauge for `buffer_id` from its
    `prometheus_exporter` sink — see the module docstring for why this replaces a
    filesystem stat on the buffer directory. Reads as 0 before the buffer has ever held
    an event (the gauge series doesn't exist yet at that point, which is a legitimate
    empty-buffer reading, not a scrape failure)."""
    with urllib.request.urlopen(metrics_url, timeout=5) as response:
        body = response.read().decode("utf-8")
    for match in _BUFFER_BYTE_SIZE_RE.finditer(body):
        if match.group("buffer_id") == buffer_id:
            return float(match.group("value"))
    return 0.0


def make_real_cycle_driver(
    *,
    rustfs_container: str,
    metrics_url: str,
    buffer_id: str,
    shipper_host: str,
    shipper_port: int,
    connections: int,
    rate_hz: float,
    poll_interval_s: float,
    max_drain_wait_s: float,
    repo_root: str,
    min_observations: int,
) -> Callable[[float], DrainCycleResult]:
    """The real driver: induces an outage on `rustfs_container` with the exact same
    `podman stop` / wait / `podman start` recipe run.sh itself uses (#385's "existing
    incident/outage scenario scaffolding, reused unmodified" acceptance criterion — this
    calls podman directly rather than shelling out to run.sh, since run.sh's outage
    window is only one step inside a much larger zero-loss run; the *recipe* is what's
    reused, run.sh itself is untouched), drives load_driver.py at the stated connections
    and rate for the outage's duration, then polls the Shipper's `buffer_id` buffer via
    `metrics_url` until it drains or `max_drain_wait_s` elapses."""

    if poll_interval_s * min_observations > 0 and poll_interval_s <= 0:
        raise ValueError("poll_interval_s must be positive")

    def driver(outage_length_s: float) -> DrainCycleResult:
        if outage_length_s < poll_interval_s * min_observations:
            raise ValueError(
                f"outage length {outage_length_s}s is too short to collect "
                f"{min_observations} samples at a {poll_interval_s}s poll interval — "
                "raise the outage length or lower the poll interval"
            )

        _podman("stop", rustfs_container)

        driver_proc = subprocess.Popen(
            [
                "uv",
                "run",
                "--frozen",
                "python3",
                os.path.join(repo_root, "tools/e2e/scripts/load_driver.py"),
                shipper_host,
                str(shipper_port),
                "--connections",
                str(connections),
                "--rate",
                str(rate_hz),
                "--duration",
                str(outage_length_s),
                "--tag",
                "dc.e2e.drain_rate",
            ],
            cwd=repo_root,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        outage_window: list[Observation] = []
        peak_bytes = 0.0
        deadline = time.monotonic() + outage_length_s
        while time.monotonic() < deadline:
            size = _buffer_bytes(metrics_url, buffer_id)
            peak_bytes = max(peak_bytes, size)
            outage_window.append(
                Observation(
                    ack_latency_s=0.0,
                    unacked_window_depth=0.0,
                    disk_buffer_bytes=size,
                    outage_declared=True,
                )
            )
            time.sleep(poll_interval_s)
        driver_proc.wait(timeout=max(poll_interval_s * 4, 30))

        _podman("start", rustfs_container)

        # A small absolute floor, not a fraction of the peak: unlike a filesystem stat
        # on the buffer directory, vector_buffer_byte_size genuinely reaches (very close
        # to) 0 once every pending event has been acked by the real Destination —
        # verified empirically while building this module.
        drained_threshold_bytes = 4_096.0
        recovery_window: list[Observation] = []
        drain_time_s: float | None = None
        recovery_start = time.monotonic()
        while time.monotonic() - recovery_start < max_drain_wait_s:
            size = _buffer_bytes(metrics_url, buffer_id)
            recovery_window.append(
                Observation(
                    ack_latency_s=0.0,
                    unacked_window_depth=0.0,
                    disk_buffer_bytes=size,
                    outage_declared=False,
                )
            )
            if size <= drained_threshold_bytes:
                drain_time_s = time.monotonic() - recovery_start
                break
            time.sleep(poll_interval_s)

        return DrainCycleResult(
            outage_length_s=outage_length_s,
            connections=connections,
            rate_hz=rate_hz,
            peak_buffer_bytes=peak_bytes,
            drain_time_s=drain_time_s,
            outage_window=tuple(outage_window),
            recovery_window=tuple(recovery_window),
        )

    return driver


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rustfs-container", required=True)
    parser.add_argument("--shipper-host", default="127.0.0.1")
    parser.add_argument("--shipper-port", type=int, default=24224)
    parser.add_argument(
        "--metrics-url",
        default="http://127.0.0.1:9598/metrics",
        help="the Shipper-under-test's prometheus_exporter scrape URL",
    )
    parser.add_argument("--buffer-id", default="to_object_storage")
    parser.add_argument("--connections", type=int, default=20)
    parser.add_argument("--rate", type=float, default=10.0, help="per-connection Records/s")
    parser.add_argument(
        "--outage-lengths",
        default="30,60",
        help="comma-separated ascending outage lengths in seconds",
    )
    parser.add_argument("--poll-interval", type=float, default=5.0)
    parser.add_argument("--max-drain-wait", type=float, default=120.0)
    parser.add_argument(
        "--disk-buffer-growth-threshold-bytes",
        type=float,
        default=500_000.0,
        help="overrides saturation_probe's default 1MB — tuned down so a short "
        "synthetic-load outage reliably produces a detectable growth trend",
    )
    parser.add_argument("--repo-root", default=os.getcwd())
    parser.add_argument("--report-json", required=True)
    parser.add_argument("--summary-txt", required=True)
    parser.add_argument("--measurements-json", required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    outage_lengths = [float(x) for x in args.outage_lengths.split(",") if x.strip()]

    driver = make_real_cycle_driver(
        rustfs_container=args.rustfs_container,
        metrics_url=args.metrics_url,
        buffer_id=args.buffer_id,
        shipper_host=args.shipper_host,
        shipper_port=args.shipper_port,
        connections=args.connections,
        rate_hz=args.rate,
        poll_interval_s=args.poll_interval,
        max_drain_wait_s=args.max_drain_wait,
        repo_root=args.repo_root,
        min_observations=SaturationBounds().min_observations,
    )
    bounds = SaturationBounds(
        disk_buffer_growth_threshold_bytes=args.disk_buffer_growth_threshold_bytes
    )

    try:
        axis_run, cycles_by_level = find_drain_rate_curve(
            driver, outage_lengths, args.connections, args.rate, bounds
        )
    except OutageMisreportedError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

    report = build_report([axis_run])
    with open(args.report_json, "w") as f:
        f.write(to_json(report))
    with open(args.summary_txt, "w") as f:
        f.write(render_summary(report))

    measurements = {
        str(level): {
            "outage_length_s": cycle.outage_length_s,
            "connections": cycle.connections,
            "rate_hz": cycle.rate_hz,
            "peak_buffer_bytes": cycle.peak_buffer_bytes,
            "drain_time_s": cycle.drain_time_s,
        }
        for level, cycle in cycles_by_level.items()
    }
    with open(args.measurements_json, "w") as f:
        json.dump(measurements, f, indent=2, sort_keys=True)

    print(render_summary(report))
    print(json.dumps(measurements, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
