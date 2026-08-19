#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Single-robot ceiling axis orchestrator (#383, part of #323's limits harness epic).

Wires single_robot_ceiling_driver.py's real `RealStackHandle` (podman exec / `ros2
param set` / a Postgres query, all real I/O — see `PodmanRealStackHandle` below) to the
*unmodified* `ramp_controller.find_knee()` and `saturation_probe.evaluate()`, then folds
the ramp result into curve_reporter.py's `AxisRun` -> `CurveReport` (#380) — the same
report format every other axis in the epic uses. Called by
run_limits_single_robot_ceiling.sh, which owns bringing the one-real-stack topology
up/down (a sibling of run_limits_two_tier.sh, #381) and shells out here for the ramp
itself, matching #323's PRD: "only the driver adapter and topology wiring are new".

Zero loss ("asserted during the limits run at sustainable load" per the PRD) is checked
once, cumulatively, by run_limits_single_robot_ceiling.sh calling the existing
verify_zero_loss.py after this script returns and a drain window lets any still-in-flight
backlog land — not after every individual clear level. Stopping and restarting the stack
between every ramp step to run verify_zero_loss.py's full check (which needs the stack
stopped so passthrough/raw/MCAP output settle, see run_limits_two_tier.sh) would reset
buffer state each time and defeat the point of a continuous ramp. Every sustainable
level's traffic is a strict subset of what that one cumulative, post-drain check covers,
so a pass proves zero loss at each of them individually; a fail names exactly which
Records never arrived.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from curve_reporter import (
    AxisRun,
    LevelResult,
    PointKind,
    ResourceSample,
    RunComposition,
    RunOutcome,
    build_report,
    render_summary,
    to_json,
)
from ramp_controller import RampOutcome, RampPolicy, RampResult, find_knee
from saturation_probe import SaturationBounds
from single_robot_ceiling_driver import (
    RealStackSample,
    SingleRobotCeilingRun,
    make_driver,
    make_probe,
)
from verify_zero_loss import psql

DEFAULT_BUFFER_PATH = "/root/.dc/e2e/buffer"
DEFAULT_LEDGER_PATH = "/root/.dc/e2e/data/workload_ledger.txt"


# --- pure parsing helpers (unit-tested against canned command output; no subprocess) ----


def parse_published_values(ledger_text: str, topic: str) -> frozenset:
    values = set()
    for line in ledger_text.splitlines():
        source, _, value = line.strip().partition(",")
        if source == topic and value.isdigit():
            values.add(int(value))
    return frozenset(values)


def parse_arrived_values(distinct_value_rows: str) -> frozenset:
    return frozenset(int(v) for v in distinct_value_rows.split() if v.strip().isdigit())


def parse_disk_bytes(du_output: str) -> float:
    stripped = du_output.strip()
    if not stripped:
        return 0.0
    return float(stripped.split()[0])


def parse_podman_stats(stats_line: str) -> tuple[float, float]:
    """`podman stats --format '{{.CPUPerc}},{{.MemPerc}}'` output -> (cpu_percent, mem_percent)."""
    stripped = stats_line.strip()
    if not stripped:
        return 0.0, 0.0
    cpu_str, _, mem_str = stripped.partition(",")
    return float(cpu_str.rstrip("%") or 0.0), float(mem_str.rstrip("%") or 0.0)


def parse_disk_percent(df_output: str) -> float:
    """`df --output=pcent <path>` output (a header line plus one value line) -> percent."""
    lines = [line.strip().rstrip("%") for line in df_output.splitlines() if line.strip()]
    if len(lines) < 2:
        return 0.0
    value = lines[-1]
    return float(value) if value.replace(".", "", 1).isdigit() else 0.0


# --- the real-I/O RealStackHandle --------------------------------------------------------


class PodmanRealStackHandle:
    """single_robot_ceiling_driver.RealStackHandle backed by real podman/ROS/Postgres —
    the only I/O this axis's driver adapter performs."""

    def __init__(
        self,
        dc_container: str,
        postgres_container: str,
        topic: str,
        buffer_path: str = DEFAULT_BUFFER_PATH,
        ledger_path: str = DEFAULT_LEDGER_PATH,
    ) -> None:
        self.dc_container = dc_container
        self.postgres_container = postgres_container
        self.topic = topic
        self.buffer_path = buffer_path
        self.ledger_path = ledger_path

    def set_rate(self, rate_hz: float) -> None:
        subprocess.run(
            [
                "podman",
                "exec",
                self.dc_container,
                "ros2",
                "param",
                "set",
                "/e2e_workload_generator",
                "rate_hz",
                str(rate_hz),
            ],
            check=True,
            capture_output=True,
            text=True,
        )

    def sample(self) -> RealStackSample:
        return RealStackSample(
            sampled_at_s=time.monotonic(),
            published_values=self._published(),
            arrived_values=self._arrived(),
            disk_buffer_bytes=self._disk_buffer_bytes(),
        )

    def sample_resources(self) -> ResourceSample:
        stats = subprocess.run(
            [
                "podman",
                "stats",
                "--no-stream",
                "--format",
                "{{.CPUPerc}},{{.MemPerc}}",
                "--filter",
                f"name={self.dc_container}",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        cpu_percent, mem_percent = parse_podman_stats(stats.stdout if stats.returncode == 0 else "")

        df = subprocess.run(
            ["podman", "exec", self.dc_container, "df", "--output=pcent", self.buffer_path],
            capture_output=True,
            text=True,
            check=False,
        )
        disk_percent = parse_disk_percent(df.stdout if df.returncode == 0 else "")

        return ResourceSample(
            cpu_percent=cpu_percent, mem_percent=mem_percent, disk_percent=disk_percent
        )

    def _published(self) -> frozenset:
        result = subprocess.run(
            ["podman", "exec", self.dc_container, "cat", self.ledger_path],
            capture_output=True,
            text=True,
            check=False,
        )
        return parse_published_values(result.stdout if result.returncode == 0 else "", self.topic)

    def _arrived(self) -> frozenset:
        rows = psql(
            self.postgres_container,
            f"SELECT DISTINCT value FROM dc_records WHERE source='{self.topic}'",
        )
        return parse_arrived_values(rows)

    def _disk_buffer_bytes(self) -> float:
        result = subprocess.run(
            ["podman", "exec", self.dc_container, "du", "-sb", self.buffer_path],
            capture_output=True,
            text=True,
            check=False,
        )
        return parse_disk_bytes(result.stdout if result.returncode == 0 else "")


# --- folding the ramp result into curve_reporter's AxisRun (pure; unit-tested) ----------


def build_axis_run(
    result: RampResult, recorded: list[tuple[float, SingleRobotCeilingRun, ResourceSample]]
) -> AxisRun:
    """Turns one find_knee() result plus the per-level runs/resource samples this
    script recorded as a side effect of driving them into curve_reporter's `AxisRun` —
    #383's "output is folded into the curve reporter's stable report format" criterion.
    Pure and independent of how `recorded` was gathered, so it's tested directly against
    fabricated ramp results."""
    points = [
        LevelResult(
            level=rate_hz,
            saturated=(
                result.outcome == RampOutcome.KNEE_FOUND and rate_hz == result.tripped_level
            ),
            kind=PointKind.MEASURED,
            composition=RunComposition(real_stacks=1, synthetic_senders=0),
            resources=resources,
        )
        for rate_hz, _run, resources in recorded
    ]
    return AxisRun(
        axis="single_robot_ceiling",
        unit="records/s",
        outcome=RunOutcome(result.outcome.value),
        highest_clear_level=result.highest_clear_level,
        tripped_level=result.tripped_level,
        points=points,
    )


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dc-container", required=True)
    parser.add_argument("--postgres-container", required=True)
    parser.add_argument(
        "--topic", default="synth00", help="the ledger-checked synth topic to ramp/sample"
    )
    parser.add_argument("--buffer-path", default=DEFAULT_BUFFER_PATH)
    parser.add_argument("--ledger-path", default=DEFAULT_LEDGER_PATH)
    parser.add_argument(
        "--levels", required=True, help="comma-separated ascending Records/s levels"
    )
    parser.add_argument("--level-duration", type=float, default=30.0, help="seconds per ramp level")
    parser.add_argument(
        "--sample-interval", type=float, default=3.0, help="seconds between polls within a level"
    )
    parser.add_argument("--report-json", default=None)
    parser.add_argument("--report-txt", default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])

    levels = [float(x) for x in args.levels.split(",") if x.strip()]
    policy = RampPolicy(levels=levels)

    handle = PodmanRealStackHandle(
        dc_container=args.dc_container,
        postgres_container=args.postgres_container,
        topic=args.topic,
        buffer_path=args.buffer_path,
        ledger_path=args.ledger_path,
    )
    driver = make_driver(
        handle, duration_s=args.level_duration, sample_interval_s=args.sample_interval
    )
    probe = make_probe(SaturationBounds())

    recorded: list[tuple[float, SingleRobotCeilingRun, ResourceSample]] = []

    def recording_driver(rate_hz: float) -> SingleRobotCeilingRun:
        run = driver(rate_hz)
        recorded.append((rate_hz, run, handle.sample_resources()))
        return run

    result = find_knee(recording_driver, probe, policy)

    report = build_report([build_axis_run(result, recorded)])
    summary = render_summary(report)
    print(summary)

    if args.report_json:
        Path(args.report_json).write_text(to_json(report))
    if args.report_txt:
        Path(args.report_txt).write_text(summary)

    print(
        json.dumps(
            {
                "ramp_outcome": result.outcome.value,
                "tripped_level": result.tripped_level,
                "highest_clear_level": result.highest_clear_level,
            }
        )
    )

    # BOUND_NOT_FOUND is a legitimate, reportable result (per ramp_controller.py's own
    # contract — never a failure by itself); only a Python-level error should fail this
    # process. Whether the ramp reached a knee is read from the report, not the exit code.
    return 0


if __name__ == "__main__":
    sys.exit(main())
