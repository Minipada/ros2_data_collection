#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Limits harness: Shipper fan-in axis (#382, the flagship axis of #323's epic).

Drives #379's ramp controller over #381's two-tier topology, using #378's synthetic
load driver as the injected driver and #377's saturation probe as the injected verdict
function, and reports through #380's curve reporter. One process handles one topology
instance (one aggregating Shipper, one set of real stacks) — `run_limits_shipper_fanin.sh`
invokes it twice, once per instance, for the unconstrained run and the deliberately
resource-constrained control the PRD's "prove the instrument itself" requirement calls
for; this module does not know which one it is beyond the `--label` it stamps on its
report.

The ramp varies only the synthetic sender count — the connection count is exactly the
"how many robots does one Shipper serve" axis (#323's PRD). A small fixed number of real
DC stacks (`--real-stacks`, default 1, matching #381's own discovered default) run
alongside every level for fidelity, per the PRD's hybrid load-generation decision.

One ramp level = a handful of short back-to-back load_driver.py sub-runs at that
connection count, each contributing one `saturation_probe.Observation` (ack-latency
p95, unacked-window depth, and the aggregating Shipper's own disk-buffer growth,
sampled via `podman exec`/`du`) to the window `saturation_probe.evaluate()` judges.
`load_driver.run()` has no mid-run introspection hook — it blocks for its whole
duration and returns one final summary — so a sustained level is approximated as
several fresh short bursts rather than one continuous connection; this is judged
sufficient for a fan-in load test, since what saturates an aggregating Shipper's ingest
listener is connection count and aggregate byte rate, not any one connection's
lifetime.

Zero loss is asserted **at every sustainable level**, not just once at the end, by
hooking straight into `ramp_controller.find_knee()`'s own driver/probe seam: the
`probe` handed to `find_knee()` first asks `saturation_probe.evaluate()` for the
verdict, and only when that verdict is NOT saturated does it also run
`verify_zero_loss.py` — "the existing verifier," reused unmodified, exactly as #381's
own two-tier composer already does — against a snapshot of the ledger-checked real
stack's output. A violation there is a hard failure of the whole ramp, immediately,
before any higher level is ever tried: a throughput figure obtained from a
configuration that was dropping Records must never be reported (#323's PRD).

Recorded discovery, in the same spirit as #381's own two: the snapshot verify_zero_loss.py
checks against is extracted from the real stack's *still-running* data volume (the
ledger-checked real stack keeps running across the whole ramp — stopping and restarting
it between every level would be far too slow, and would also break the "one continuous
real-stack run backs every sustainable level" property this check exists to establish).
A named-volume read while another container is still appending to the same files races
that container, and one instance of that race has a precise, diagnosed cause rather than
a vague "torn write": `entrypoint.sh` starts `dc_mcap_writer` with `--max-duration-secs
15`, and (per that writer's own `RotatingMcapWriter._open_next_file()` docstring) a
`.mcap` file only becomes readable once its 15s rotation finishes and the writer calls
`Writer.finish()` — the handful of most-recent Records always sit in a *currently open,
unfinished* file that `mcap_summary.py` cannot parse at all yet. A live snapshot taken
mid-rotation therefore reports those Records as "never reached dc_mcap_writer" even
though they are sitting safely on disk, about to become readable a few seconds later.
Verified empirically against a real run: a level's check failed with a clean,
well-formed "ZERO-LOSS VERIFICATION FAILED" (not a parse crash) that
the exact same check, re-run against the same still-live containers a bit later, no
longer reproduced. So `verify_zero_loss_at_level()` retries *any* failure shape once,
after a pause comfortably longer than that 15s rotation window
(`_ZERO_LOSS_RETRY_SLEEP_S`), before treating it as a hard failure — a genuine violation
is expected to reproduce on both attempts (well past any rotation boundary), so this
costs nothing on real loss, and a failure is never distinguished from a real violation
beyond that single retry, so it can never silently pass off as "verification didn't
really run."
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import curve_reporter
import ramp_controller
import saturation_probe
from load_driver import LoadDriverConfig, run_sync

SCRIPT_DIR = Path(__file__).resolve().parent

# The aggregating Shipper's `sinks.to_postgres.buffer.max_size`
# (run_limits_two_tier.sh's rendered aggregator_vector.toml) — the denominator disk-buffer
# growth is expressed as a percent of, for the curve report's ResourceSample. Not itself a
# saturation threshold (saturation_probe judges growth, never raw utilisation — see its own
# docstring for why).
DEFAULT_AGG_BUFFER_MAX_BYTES = 268_435_488

DEFAULT_LEVELS = (25, 50, 100, 200, 400)
DEFAULT_SYNTH_RATE_HZ = 5.0
DEFAULT_SUB_WINDOW_SECONDS = 5.0
DEFAULT_SUB_WINDOWS_PER_LEVEL = 5

# entrypoint.sh runs dc_mcap_writer with `--max-duration-secs 15` — comfortably longer
# than that, so a retry always lands after the rotation boundary that made the previous
# attempt's most-recent Records unreadable (see the module docstring's discovery).
_ZERO_LOSS_RETRY_SLEEP_S = 18.0


class ZeroLossViolationError(RuntimeError):
    """Raised when verify_zero_loss.py reports a real violation (not a transient
    read race) — a hard, unrecoverable failure of the whole ramp."""


def _podman_exec(container: str, *cmd: str) -> str:
    result = subprocess.run(
        ["podman", "exec", container, *cmd], capture_output=True, text=True, check=False
    )
    return result.stdout.strip() if result.returncode == 0 else ""


def sample_disk_buffer_bytes(agg_container: str, data_dir: str = "/var/lib/vector") -> float:
    """Total bytes under the aggregating Shipper's Vector data dir — dominated by
    `sinks.to_postgres.buffer`, the sink carrying the at-least-once guarantee real
    stacks' Records depend on. `du` failing (container not up yet) reads as 0 bytes,
    not an error — sampled only ever alongside other observations that would themselves
    fail loudly if the container were actually gone."""
    out = _podman_exec(agg_container, "du", "-sb", data_dir)
    if not out:
        return 0.0
    try:
        return float(out.split()[0])
    except (ValueError, IndexError):
        return 0.0


def sample_resource_usage(container: str) -> tuple[float, float]:
    """(cpu_percent, mem_percent) for `container` via `podman stats`, mirroring
    measure_resources.sh's own field choice. (0.0, 0.0) if the container has no stats
    available right now (e.g. still starting) — informational, never a gate by itself."""
    result = subprocess.run(
        [
            "podman",
            "stats",
            "--no-stream",
            "--format",
            "{{.CPUPerc}},{{.MemPerc}}",
            "--filter",
            f"name={container}",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    line = result.stdout.strip()
    if result.returncode != 0 or not line:
        return (0.0, 0.0)
    try:
        cpu_s, mem_s = line.split(",")
        return (float(cpu_s.rstrip("%")), float(mem_s.rstrip("%")))
    except ValueError:
        return (0.0, 0.0)


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = max(0, min(len(ordered) - 1, int(round(pct / 100.0 * (len(ordered) - 1)))))
    return ordered[idx]


@dataclass(frozen=True)
class LevelConfig:
    agg_host: str
    agg_port: int
    synth_rate_hz: float = DEFAULT_SYNTH_RATE_HZ
    sub_window_seconds: float = DEFAULT_SUB_WINDOW_SECONDS
    sub_windows_per_level: int = DEFAULT_SUB_WINDOWS_PER_LEVEL
    tag: str = "dc.e2e.limits_synth"


def drive_level(
    level: float, agg_container: str, cfg: LevelConfig
) -> list[saturation_probe.Observation]:
    """The ramp controller's injected `driver`: runs `cfg.sub_windows_per_level` short
    load_driver bursts at `level` synthetic connections back to back, turning each into
    one Observation. Returns the whole window, oldest first, for the probe to judge."""
    window: list[saturation_probe.Observation] = []
    for _ in range(cfg.sub_windows_per_level):
        driver_config = LoadDriverConfig(
            host=cfg.agg_host,
            port=cfg.agg_port,
            connection_count=int(level),
            record_rate_hz=cfg.synth_rate_hz,
            duration_s=cfg.sub_window_seconds,
            tag=cfg.tag,
        )
        result = run_sync(driver_config)
        window.append(
            saturation_probe.Observation(
                ack_latency_s=_percentile(result.ack_latencies_s(), 95),
                unacked_window_depth=float(len(result.unacked_chunk_ids)),
                disk_buffer_bytes=sample_disk_buffer_bytes(agg_container),
                outage_declared=False,
            )
        )
    return window


@dataclass(frozen=True)
class VerifyConfig:
    verify_script: Path
    dc_image: str
    data_volume: str
    postgres_container: str
    num_synth_topics: int
    run_dir: Path
    label: str


def _podman_cat(dc_image: str, data_volume: str, path_in_container: str) -> str:
    result = subprocess.run(
        [
            "podman",
            "run",
            "--rm",
            "--entrypoint",
            "bash",
            "-v",
            f"{data_volume}:/vol:ro",
            dc_image,
            "-c",
            f"cat {path_in_container} 2>/dev/null || true",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout


def _extract_snapshot(cfg: VerifyConfig, level: float) -> dict[str, Path]:
    prefix = cfg.run_dir / f"{cfg.label}_level_{level}"
    paths = {
        "ledger": prefix.with_suffix(".ledger.txt"),
        "passthrough": prefix.with_suffix(".passthrough.ndjson"),
        "raw": prefix.with_suffix(".raw.ndjson"),
    }
    paths["ledger"].write_text(
        _podman_cat(cfg.dc_image, cfg.data_volume, "/vol/workload_ledger.txt")
    )
    paths["passthrough"].write_text(
        _podman_cat(cfg.dc_image, cfg.data_volume, "/vol/passthrough/records.ndjson")
    )
    paths["raw"].write_text(_podman_cat(cfg.dc_image, cfg.data_volume, "/vol/raw/records.ndjson"))

    mcap = subprocess.run(
        [
            "podman",
            "run",
            "--rm",
            "--entrypoint",
            "python3",
            "-v",
            f"{cfg.data_volume}:/vol:ro",
            cfg.dc_image,
            "/opt/e2e/mcap_summary.py",
            "/vol/mcap",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    mcap_path = prefix.with_suffix(".mcap_summary.json")
    mcap_path.write_text(mcap.stdout if mcap.stdout.strip() else "{}")
    paths["mcap"] = mcap_path
    return paths


def _run_verifier_once(
    cfg: VerifyConfig, level: float, paths: dict[str, Path]
) -> subprocess.CompletedProcess:
    report_path = cfg.run_dir / f"{cfg.label}_level_{level}.verification_report.json"
    return subprocess.run(
        [
            sys.executable,
            str(cfg.verify_script),
            "--postgres-container",
            cfg.postgres_container,
            "--num-synth-topics",
            str(cfg.num_synth_topics),
            "--ledger-file",
            str(paths["ledger"]),
            "--passthrough-file",
            str(paths["passthrough"]),
            "--mcap-summary-file",
            str(paths["mcap"]),
            "--raw-file",
            str(paths["raw"]),
            "--report",
            str(report_path),
        ],
        capture_output=True,
        text=True,
        check=False,
    )


def verify_zero_loss_at_level(level: float, cfg: VerifyConfig) -> None:
    """Runs `verify_zero_loss.py` against a live snapshot of the ledger-checked real
    stack's output. Raises ZeroLossViolationError on any failure that survives one
    retry — never a silent skip. See the module docstring for the diagnosed
    live-snapshot/MCAP-rotation race this retry exists to absorb."""
    for attempt in range(2):
        paths = _extract_snapshot(cfg, level)
        proc = _run_verifier_once(cfg, level, paths)
        if proc.returncode == 0:
            return
        if attempt == 1:
            raise ZeroLossViolationError(
                f"verify_zero_loss.py failed at level {level} ({cfg.label}), "
                f"attempt {attempt + 1}/2:\nSTDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
            )
        time.sleep(_ZERO_LOSS_RETRY_SLEEP_S)  # clears the MCAP rotation boundary race


def build_axis_run(
    result: ramp_controller.RampResult,
    levels_resources: dict[float, tuple[curve_reporter.ResourceSample, bool]],
    real_stacks: int,
    label: str,
) -> curve_reporter.AxisRun:
    outcome = (
        curve_reporter.RunOutcome.KNEE_FOUND
        if result.outcome == ramp_controller.RampOutcome.KNEE_FOUND
        else curve_reporter.RunOutcome.BOUND_NOT_FOUND
    )
    points = []
    for level in result.levels_tried:
        resources, saturated = levels_resources[level]
        points.append(
            curve_reporter.LevelResult(
                level=level,
                saturated=saturated,
                kind=curve_reporter.PointKind.MEASURED,
                composition=curve_reporter.RunComposition(
                    real_stacks=real_stacks, synthetic_senders=int(level)
                ),
                resources=resources,
            )
        )
    return curve_reporter.AxisRun(
        axis=f"shipper_fan_in_{label}",
        unit="synthetic connections (robots)",
        outcome=outcome,
        highest_clear_level=result.highest_clear_level,
        tripped_level=result.tripped_level,
        points=points,
    )


def closing_sentence(
    result: ramp_controller.RampResult, real_stacks: int, synth_rate_hz: float
) -> str:
    """#323's PRD's required single defensible sentence, from an UNCONSTRAINED run:
    'one aggregating Shipper sustains N robots at M Records per second before applying
    backpressure, measured with X real stacks and Y synthetic senders, extrapolated
    beyond Z.' Never fabricates a ceiling: BOUND_NOT_FOUND states plainly that the true
    ceiling lies somewhere past the highest level tested, with no number invented for it."""
    if result.outcome == ramp_controller.RampOutcome.BOUND_NOT_FOUND:
        highest = result.highest_clear_level or 0
        return (
            f"One aggregating Shipper sustains at least {real_stacks + int(highest)} robots "
            f"({real_stacks} real + {int(highest)} synthetic) at {synth_rate_hz} Records/s "
            f"per robot without applying backpressure — the ramp never saturated within "
            f"the tested range, so no ceiling is reported beyond {int(highest)} synthetic "
            f"senders (bound not found; never fabricated)."
        )
    highest_clear = result.highest_clear_level
    synthetic_at_knee = int(highest_clear) if highest_clear is not None else 0
    return (
        f"One aggregating Shipper sustains {real_stacks + synthetic_at_knee} robots "
        f"({real_stacks} real + {synthetic_at_knee} synthetic) at {synth_rate_hz} "
        f"Records/s per robot before applying backpressure, measured with {real_stacks} "
        f"real stack(s) and {synthetic_at_knee} synthetic sender(s); saturation observed "
        f"at {int(result.tripped_level)} synthetic senders (not extrapolated — the knee "
        f"itself was measured, not projected)."
    )


def run(args: argparse.Namespace) -> ramp_controller.RampResult:
    run_dir = Path(args.run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)

    level_cfg = LevelConfig(
        agg_host=args.agg_host,
        agg_port=args.agg_port,
        synth_rate_hz=args.synth_rate_hz,
        sub_window_seconds=args.sub_window_seconds,
        sub_windows_per_level=args.sub_windows_per_level,
    )
    verify_cfg = VerifyConfig(
        verify_script=SCRIPT_DIR / "verify_zero_loss.py",
        dc_image=args.dc_image,
        data_volume=args.data_volume,
        postgres_container=args.postgres_container,
        num_synth_topics=args.num_synth_topics,
        run_dir=run_dir,
        label=args.label,
    )
    bounds = saturation_probe.SaturationBounds()
    levels_resources: dict[float, tuple[curve_reporter.ResourceSample, bool]] = {}

    def driver(level: float) -> list[saturation_probe.Observation]:
        print(
            f"[fanin/{args.label}] level {int(level)}: driving {args.sub_windows_per_level} "
            f"sub-window(s) of {args.sub_window_seconds}s at {int(level)} synthetic connections",
            flush=True,
        )
        return drive_level(level, args.agg_container, level_cfg)

    levels_seen: list[float] = []

    def probe_and_record(window: list[saturation_probe.Observation]) -> saturation_probe.Verdict:
        level = levels_seen[-1]
        verdict = saturation_probe.evaluate(window, bounds)
        cpu, mem = sample_resource_usage(args.agg_container)
        disk_bytes = window[-1].disk_buffer_bytes if window else 0.0
        levels_resources[level] = (
            curve_reporter.ResourceSample(
                cpu_percent=cpu,
                mem_percent=mem,
                disk_percent=(disk_bytes / args.agg_buffer_max_bytes) * 100.0,
            ),
            verdict.saturated,
        )
        print(
            f"[fanin/{args.label}] level {int(level)}: {verdict.status.value} ({verdict.reason})",
            flush=True,
        )
        if not verdict.saturated:
            print(
                f"[fanin/{args.label}] level {int(level)}: clear — asserting zero loss "
                f"via verify_zero_loss.py before continuing",
                flush=True,
            )
            verify_zero_loss_at_level(level, verify_cfg)
            print(f"[fanin/{args.label}] level {int(level)}: PASS zero-loss", flush=True)
        return verdict

    def tracking_driver(level: float) -> list[saturation_probe.Observation]:
        levels_seen.append(level)
        return driver(level)

    policy = ramp_controller.RampPolicy(levels=tuple(float(v) for v in args.levels))
    result = ramp_controller.find_knee(tracking_driver, probe_and_record, policy)

    axis_run = build_axis_run(result, levels_resources, args.real_stacks, args.label)
    report = curve_reporter.build_report([axis_run])

    Path(args.report_json).write_text(curve_reporter.to_json(report))
    sentence = closing_sentence(result, args.real_stacks, args.synth_rate_hz)
    summary = curve_reporter.render_summary(report) + "\nClosing statement:\n" + sentence + "\n"
    Path(args.report_txt).write_text(summary)
    print(summary)

    print(
        json.dumps(
            {
                "label": args.label,
                "outcome": result.outcome.value,
                "highest_clear_level": result.highest_clear_level,
                "tripped_level": result.tripped_level,
            }
        )
    )
    return result


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True, help="e.g. 'unconstrained' or 'constrained'")
    parser.add_argument("--agg-host", required=True)
    parser.add_argument("--agg-port", type=int, required=True)
    parser.add_argument("--agg-container", required=True)
    parser.add_argument("--postgres-container", required=True)
    parser.add_argument(
        "--data-volume", required=True, help="the ledger-checked real stack's data volume"
    )
    parser.add_argument("--dc-image", required=True)
    parser.add_argument("--real-stacks", type=int, default=1)
    parser.add_argument("--num-synth-topics", type=int, default=14)
    parser.add_argument("--levels", type=int, nargs="+", default=list(DEFAULT_LEVELS))
    parser.add_argument("--synth-rate-hz", type=float, default=DEFAULT_SYNTH_RATE_HZ)
    parser.add_argument("--sub-window-seconds", type=float, default=DEFAULT_SUB_WINDOW_SECONDS)
    parser.add_argument("--sub-windows-per-level", type=int, default=DEFAULT_SUB_WINDOWS_PER_LEVEL)
    parser.add_argument("--agg-buffer-max-bytes", type=float, default=DEFAULT_AGG_BUFFER_MAX_BYTES)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--report-json", required=True)
    parser.add_argument("--report-txt", required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    try:
        run(args)
    except ZeroLossViolationError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
