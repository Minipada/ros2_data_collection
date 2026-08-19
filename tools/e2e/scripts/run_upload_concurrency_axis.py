#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Uploader concurrency axis (#384, part of #323's limits harness epic): ramps the
number of concurrent Bridge/Uploader processes against one shared object-storage
Destination until verify-then-delete falls behind, reporting the last sustainable
concurrency level.

Run through tools/e2e/scripts/run_limits_upload_concurrency.sh, which stands up the
shared Postgres + RustFS this script points every level's Bridge containers at and tears
them down afterwards; this script owns only the ramp itself, calling
tools/e2e/scripts/ramp_controller.py's find_knee() unchanged (#384's acceptance
criterion) with this axis's own driver and
tools/e2e/scripts/upload_saturation_probe.py's evaluate() as the probe, then feeding the
result to tools/e2e/scripts/curve_reporter.py's build_report() in its stable format.

The Uploader has no internal concurrency knob (dc_bridge/src/uploader/uploader.hpp:
"Synchronous (aws-sdk-cpp is blocking) — no async runtime", one uploader_thread_ per
Bridge process) — confirmed by reading dc_bridge/src/bridge_node.cpp's full param
schema, which declares none. Ramping "concurrency" therefore means starting N separate
Bridge/Uploader processes, each with its own serial uploader thread, all pointed at the
same shared RustFS/Postgres — the same topology run_limits_two_tier.sh (#381) already
proved works for Files, whose header recorded that Files bypass that scenario's two-tier
Shipper-relay chain entirely (the Uploader's own AWS SDK/libpq calls, no Vector hop) and
left "true two-tier coverage" for Files as a follow-up; this script is the single-tier
concurrency-and-custody axis #381 punted, not that follow-up.

Two observables stand in for saturation_probe.py's Shipper-specific ack
latency/unacked-window/disk-buffer signals, since Files never touch the Shipper at all
(see upload_saturation_probe.py's own docstring for why a File-specific probe exists
instead of reusing that one): the Bridge's own `~/ready` service, which already exposes
its upload queue depth (#265's "cheap observability hook", bridge_node.cpp) as an
existing observability hook — no DC code changed to add it — and the shared `dc_files`
table's verify/delete rows (ADR-0005), which this script queries via
verify_zero_loss.py's own podman-exec `psql()`/`scalar_int()` helpers rather than a
second Postgres client.

At every level actually driven, before that level's containers are torn down, two hard
gates run against `dc_files` — never merged into the saturation verdict, since they are
correctness properties, not backpressure signals:

1. **Delete-after-verify ordering** — no `deleted=true` row for a local_path may exist
   without an earlier-or-equal `uploaded=true, deleted=false` row for the same path.
2. **Group completion correctness** — every `group_complete` marker for the camera group
   names the expected file count, and at least one landed.

A violation raises immediately and aborts the whole ramp (fails loudly, per #323's PRD:
"a verification that silently did not run must never look identical to one that
passed") rather than being folded into "saturated" — a concurrency level can be
correctness-broken without ever looking like a backpressure ceiling, and the two must
never be confused.
"""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field

import curve_reporter
import ramp_controller
import upload_saturation_probe
import verify_zero_loss

READY_SERVICE = "/dc_bridge/ready"


def log(msg: str) -> None:
    print(f"[upload-axis {time.strftime('%H:%M:%S', time.gmtime())}] {msg}", flush=True)


def _podman(*args: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["podman", *args], capture_output=True, text=True, check=True, timeout=timeout
    )


def _queue_depth(container: str, attempts: int = 3) -> int:
    """Sums Bridge's own `~/ready` observability hook's upload queue depth for one
    container. Raises only once every attempt has failed — `ros2 service call`'s own
    DDS-discovery step occasionally takes longer than one 20s attempt under load
    (verified empirically), and a transient CLI hiccup reading a monitoring probe must
    not abort an otherwise-healthy ramp the way a real infra failure should."""
    last_error: str = ""
    for attempt in range(attempts):
        try:
            result = subprocess.run(
                [
                    "podman",
                    "exec",
                    container,
                    "bash",
                    "-lc",
                    "source /root/ws/install/setup.bash >/dev/null 2>&1; "
                    f"ros2 service call {READY_SERVICE} std_srvs/srv/Trigger '{{}}'",
                ],
                capture_output=True,
                text=True,
                check=False,
                timeout=20,
            )
        except subprocess.TimeoutExpired:
            last_error = f"attempt {attempt + 1}/{attempts} timed out after 20s"
            continue
        if result.returncode != 0:
            last_error = f"attempt {attempt + 1}/{attempts} failed: {result.stderr.strip()}"
            continue
        match = re.search(r"upload queue depth:\s*(\d+)", result.stdout)
        if match is None:
            last_error = (
                f"attempt {attempt + 1}/{attempts} carried no upload queue depth figure "
                f"(stdout: {result.stdout.strip()!r})"
            )
            continue
        return int(match.group(1))
    raise RuntimeError(
        f"readiness probe failed on {container} after {attempts} attempt(s): {last_error}"
    )


def _wait_for_containers_ready(names: list[str], timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    for name in names:
        while True:
            try:
                _queue_depth(name)
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"{name} never became ready (upload queue depth probe) within {timeout_s}s"
                    ) from None
                time.sleep(1)


def _volume_names(prefix: str, n: int, i: int) -> tuple[str, str]:
    return f"{prefix}_buffer_{n}_{i}", f"{prefix}_data_{n}_{i}"


def _start_level_containers(
    image: str,
    network: str,
    params_file: str,
    stack_prefix: str,
    volume_prefix: str,
    n: int,
    camera_period_s: float,
    stagger_s: float,
) -> list[str]:
    names = []
    for i in range(n):
        stack_id = f"l{n}-{i}"
        name = f"{stack_prefix}-{stack_id}"
        buf_vol, data_vol = _volume_names(volume_prefix, n, i)
        _podman("volume", "create", buf_vol)
        _podman("volume", "create", data_vol)
        _podman(
            "run",
            "-d",
            "--network",
            network,
            "--name",
            name,
            "-e",
            f"DC_E2E_CAMERA_PERIOD_S={camera_period_s}",
            # Makes each container's captured-file local_path genuinely unique — see
            # params/e2e_limits_upload_params.yaml's own comment on
            # save_local_base_path for why this is load-bearing, not cosmetic.
            "-e",
            f"DC_E2E_STACK_ID={stack_id}",
            "-v",
            f"{buf_vol}:/root/.dc/e2e/buffer",
            "-v",
            f"{data_vol}:/root/.dc/e2e/data",
            "-v",
            f"{params_file}:/opt/e2e/e2e_params.yaml:ro",
            image,
        )
        names.append(name)
        # Staggered, not all at once: same reasoning as run_limits_two_tier.sh — reduces
        # simultaneous ROS/DDS-discovery contention across N concurrently-starting stacks.
        if i < n - 1:
            time.sleep(stagger_s)
    return names


def _stop_level_containers(names: list[str], volume_prefix: str, n: int, run_dir: str) -> None:
    for name in names:
        try:
            with open(f"{run_dir}/{name}.log", "w") as f:
                subprocess.run(
                    ["podman", "logs", name], stdout=f, stderr=subprocess.STDOUT, check=False
                )
        except OSError:
            pass
    if names:
        subprocess.run(["podman", "rm", "-f", *names], check=False, capture_output=True)
    for i in range(n):
        buf_vol, data_vol = _volume_names(volume_prefix, n, i)
        for v in (buf_vol, data_vol):
            subprocess.run(["podman", "volume", "rm", v], check=False, capture_output=True)


def _outstanding_verified_count(pg_container: str, level_start: float) -> int:
    """Files verified since `level_start` with no matching delete row yet.

    `dc_files` is an append-only event log (Vector's postgres sink does INSERT, not
    UPDATE — ADR-0005): the row a file's upload/verify writes never changes, and a later
    delete is a *separate* row for the same local_path, not an update of the first one.
    Naively counting `uploaded=true AND deleted=false` rows therefore counts every
    verify event ever recorded, including ones already long deleted — always growing,
    never draining, regardless of whether delete-then-verify is actually keeping up
    (caught empirically running this script for real: a healthy single-Uploader run
    that logged "1 verified, 1 deleted" every cycle still showed this figure climbing
    monotonically before the fix). The backlog that actually matters is verified files
    with *no later delete row at all* for the same local_path.
    """
    return verify_zero_loss.scalar_int(
        pg_container,
        f"""
        SELECT count(*) FROM (
          SELECT DISTINCT local_path FROM dc_files
          WHERE kind='file_status' AND uploaded=true AND deleted=false
            AND updated_at >= {level_start}
        ) u
        WHERE NOT EXISTS (
          SELECT 1 FROM dc_files d
          WHERE d.kind='file_status' AND d.local_path = u.local_path AND d.deleted=true
        )
        """,
    )


def _assert_delete_after_verify(pg_container: str, level_start: float) -> None:
    """Every `deleted=true` row must have an earlier-or-equal `uploaded=true,
    deleted=false` row for the same local_path — the empirical, DB-level proof that
    delete-after-verify held under N concurrently-uploading processes, not just an
    assumption that a single process's own in-order code path generalizes."""
    violations = verify_zero_loss.scalar_int(
        pg_container,
        f"""
        SELECT count(*) FROM dc_files d
        WHERE d.kind='file_status' AND d.deleted=true AND d.updated_at >= {level_start}
          AND NOT EXISTS (
            SELECT 1 FROM dc_files u
            WHERE u.kind='file_status' AND u.local_path = d.local_path
              AND u.uploaded=true AND u.deleted=false AND u.updated_at <= d.updated_at
          )
        """,
    )
    if violations > 0:
        raise RuntimeError(
            f"{violations} file(s) at this level were deleted with no preceding "
            "verified-upload row — delete fired before/without verification"
        )


def _assert_group_completion(
    pg_container: str, level_start: float, expected_file_count: int
) -> None:
    complete_count = verify_zero_loss.scalar_int(
        pg_container,
        "SELECT count(*) FROM dc_files WHERE kind='group_complete' AND group_name='camera' "
        f"AND updated_at >= {level_start}",
    )
    if complete_count == 0:
        raise RuntimeError("zero group_complete markers landed for the camera group at this level")
    bad_count = verify_zero_loss.scalar_int(
        pg_container,
        "SELECT count(*) FROM dc_files WHERE kind='group_complete' AND group_name='camera' "
        f"AND updated_at >= {level_start} AND file_count != {expected_file_count}",
    )
    if bad_count > 0:
        raise RuntimeError(
            f"{bad_count} group_complete marker(s) at this level reported a file_count "
            f"other than the expected {expected_file_count}"
        )


@dataclass
class LevelRun:
    level: float
    window: list = field(default_factory=list)


def build_driver(
    *,
    image: str,
    network: str,
    pg_container: str,
    params_file: str,
    stack_prefix: str,
    volume_prefix: str,
    run_dir: str,
    camera_period_s: float,
    steady_state_s: float,
    sample_interval_s: float,
    startup_stagger_s: float,
    startup_timeout_s: float,
    expected_file_count: int,
) -> tuple[ramp_controller.Driver, dict[float, LevelRun]]:
    runs: dict[float, LevelRun] = {}
    min_observations = upload_saturation_probe.SaturationBounds().min_observations
    sample_count = max(min_observations, math.ceil(steady_state_s / sample_interval_s))

    def driver(level: float) -> list:
        n = int(level)
        log(f"level {level}: starting {n} Bridge/Uploader container(s)")
        names = _start_level_containers(
            image,
            network,
            params_file,
            stack_prefix,
            volume_prefix,
            n,
            camera_period_s,
            startup_stagger_s,
        )
        try:
            _wait_for_containers_ready(names, startup_timeout_s)
            level_start = time.time()
            window = []
            log(f"level {level}: sampling {sample_count} observation(s) every {sample_interval_s}s")
            for _ in range(sample_count):
                time.sleep(sample_interval_s)
                queue_depth = sum(_queue_depth(name) for name in names)
                backlog = _outstanding_verified_count(pg_container, level_start)
                window.append(
                    upload_saturation_probe.Observation(
                        queue_depth=queue_depth, verify_delete_backlog=backlog
                    )
                )
                log(
                    f"level {level}: observation queue_depth={queue_depth} "
                    f"verify_delete_backlog={backlog}"
                )
            verdict = upload_saturation_probe.evaluate(window)
            log(
                f"level {level}: verdict={verdict.status.value} reason={verdict.reason!r} "
                f"figures={verdict.figures}"
            )
            log(f"level {level}: asserting delete-after-verify ordering and group completion")
            _assert_delete_after_verify(pg_container, level_start)
            _assert_group_completion(pg_container, level_start, expected_file_count)
            runs[level] = LevelRun(level=level, window=window)
            return window
        finally:
            _stop_level_containers(names, volume_prefix, n, run_dir)
            time.sleep(2)

    return driver, runs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", default="dc-e2e:latest")
    parser.add_argument("--network", required=True)
    parser.add_argument("--postgres-container", required=True)
    parser.add_argument("--params-file", required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--stack-prefix", default="dc-e2e-upload-stack")
    parser.add_argument("--volume-prefix", default="dc_e2e_upload")
    parser.add_argument(
        "--levels", default="1,2,4,8", help="comma-separated ascending concurrency levels"
    )
    parser.add_argument("--camera-period-s", type=float, default=2.0)
    parser.add_argument("--steady-state-seconds", type=float, default=60.0)
    parser.add_argument("--sample-interval-seconds", type=float, default=5.0)
    parser.add_argument("--startup-stagger-seconds", type=float, default=3.0)
    parser.add_argument("--startup-timeout-seconds", type=float, default=90.0)
    parser.add_argument("--expected-file-count", type=int, default=1)
    parser.add_argument("--report", default=None, help="write the curve_reporter JSON report here")
    args = parser.parse_args()

    levels = tuple(float(x) for x in args.levels.split(","))
    policy = ramp_controller.RampPolicy(levels=levels)

    driver, runs = build_driver(
        image=args.image,
        network=args.network,
        pg_container=args.postgres_container,
        params_file=args.params_file,
        stack_prefix=args.stack_prefix,
        volume_prefix=args.volume_prefix,
        run_dir=args.run_dir,
        camera_period_s=args.camera_period_s,
        steady_state_s=args.steady_state_seconds,
        sample_interval_s=args.sample_interval_seconds,
        startup_stagger_s=args.startup_stagger_seconds,
        startup_timeout_s=args.startup_timeout_seconds,
        expected_file_count=args.expected_file_count,
    )

    try:
        result = ramp_controller.find_knee(driver, upload_saturation_probe.evaluate, policy)
    except Exception as exc:  # any failure here is a hard scenario failure
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

    points = []
    for level in result.levels_tried:
        run = runs[level]
        verdict = upload_saturation_probe.evaluate(run.window)
        points.append(
            curve_reporter.LevelResult(
                level=level,
                saturated=verdict.saturated,
                kind=curve_reporter.PointKind.MEASURED,
                composition=curve_reporter.RunComposition(
                    real_stacks=int(level), synthetic_senders=0
                ),
            )
        )

    axis_run = curve_reporter.AxisRun(
        axis="upload_concurrency",
        unit="concurrent Bridge/Uploader processes",
        outcome=curve_reporter.RunOutcome(result.outcome.value),
        highest_clear_level=result.highest_clear_level,
        tripped_level=result.tripped_level,
        points=points,
    )
    report = curve_reporter.build_report([axis_run])
    print(curve_reporter.render_summary(report))
    if args.report:
        with open(args.report, "w") as f:
            f.write(curve_reporter.to_json(report))

    log(f"PASS: ramp completed cleanly, outcome={result.outcome.value}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
