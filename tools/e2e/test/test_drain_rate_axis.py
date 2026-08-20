# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for tools/e2e/scripts/drain_rate_axis.py (#385): the pure parts
(evaluate_cycle, build_axis_run, find_drain_rate_curve), driven entirely with synthetic
DrainCycleResult data and fake drivers — no containers, no subprocess, no network. The
real podman/load_driver.py orchestration in make_real_cycle_driver()/main() is exercised
by tools/e2e/scripts/run_limits_drain_rate.sh instead, matching the epic's "the topology
composer and the scenario script are not unit-tested; running them is the test" decision.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

from curve_reporter import PointKind, RunOutcome
from drain_rate_axis import (
    DrainCycleResult,
    OutageMisreportedError,
    build_axis_run,
    evaluate_cycle,
    find_drain_rate_curve,
)
from ramp_controller import RampOutcome, RampPolicy, RampResult, find_knee
from saturation_probe import Observation, SaturationBounds


def _observations(n: int, *, disk_buffer_bytes, outage_declared: bool) -> tuple[Observation, ...]:
    """`disk_buffer_bytes` may be a constant or a callable(i) -> float, so tests can
    describe growth/shrink trends without repeating an Observation() call per sample."""
    values = disk_buffer_bytes if callable(disk_buffer_bytes) else (lambda i: disk_buffer_bytes)
    return tuple(
        Observation(
            ack_latency_s=0.0,
            unacked_window_depth=0.0,
            disk_buffer_bytes=values(i),
            outage_declared=outage_declared,
        )
        for i in range(n)
    )


_GROWTH_STEP_BYTES = 500_000.0  # crosses SaturationBounds' default 1MB growth threshold
_PEAK_BYTES = 3_000_000.0


def _growing_outage_cycle(outage_length_s: float, drain_time_s: float | None) -> DrainCycleResult:
    """A cycle whose buffer visibly grows during the (correctly declared) outage —
    growth that *would* trip the disk-buffer signal on its own, past
    SaturationBounds' default threshold, which is what makes "not misreported while
    outage_declared=True" a real proof rather than a trend too small to trip either
    way — and whose recovery window either shrinks back to ~0 (drained) or stays flat
    at the peak (not drained), depending on `drain_time_s`."""
    outage_window = _observations(
        6, disk_buffer_bytes=lambda i: _GROWTH_STEP_BYTES * (i + 1), outage_declared=True
    )
    if drain_time_s is not None:
        recovery_window = _observations(
            6,
            disk_buffer_bytes=lambda i: _PEAK_BYTES - _GROWTH_STEP_BYTES * i,
            outage_declared=False,
        )
    else:
        recovery_window = _observations(6, disk_buffer_bytes=_PEAK_BYTES, outage_declared=False)
    return DrainCycleResult(
        outage_length_s=outage_length_s,
        connections=20,
        rate_hz=10.0,
        peak_buffer_bytes=_PEAK_BYTES,
        drain_time_s=drain_time_s,
        outage_window=outage_window,
        recovery_window=recovery_window,
    )


# --- evaluate_cycle: the outage-aware-signal acceptance criterion -------------------


def test_a_growing_buffer_during_a_declared_outage_is_not_misreported_as_saturation():
    cycle = _growing_outage_cycle(outage_length_s=30.0, drain_time_s=5.0)

    verdict = evaluate_cycle(cycle)

    assert verdict.outage_verdict.saturated is False
    assert verdict.saturated is False


def test_a_backlog_that_drains_after_the_outage_is_reported_not_saturated():
    cycle = _growing_outage_cycle(outage_length_s=30.0, drain_time_s=12.0)

    verdict = evaluate_cycle(cycle)

    assert verdict.saturated is False
    assert verdict.recovery_verdict is not None


def test_a_backlog_that_never_drains_is_reported_saturated_even_if_the_buffer_only_plateaus():
    # Recovery window is flat (600_000 the whole way through) -- growth-based trend
    # detection alone would call this "not saturated" (no growth), which would be wrong:
    # the backlog never actually cleared. drain_time_s=None must override that.
    cycle = _growing_outage_cycle(outage_length_s=30.0, drain_time_s=None)

    verdict = evaluate_cycle(cycle)

    assert verdict.saturated is True
    assert verdict.recovery_verdict is None
    assert "did not drain" in verdict.reason


def test_a_still_growing_recovery_window_is_reported_saturated():
    outage_window = _observations(
        6, disk_buffer_bytes=lambda i: _GROWTH_STEP_BYTES * (i + 1), outage_declared=True
    )
    still_growing_recovery = _observations(
        6, disk_buffer_bytes=lambda i: _PEAK_BYTES + _GROWTH_STEP_BYTES * i, outage_declared=False
    )
    cycle = DrainCycleResult(
        outage_length_s=30.0,
        connections=20,
        rate_hz=10.0,
        peak_buffer_bytes=_PEAK_BYTES,
        drain_time_s=90.0,  # a caller-supplied "it eventually stopped growing" value...
        outage_window=outage_window,
        recovery_window=still_growing_recovery,
    )

    verdict = evaluate_cycle(cycle)

    # ...is irrelevant: the recovery window itself still shows growth, so the disk-buffer
    # signal (not drain_time_s) is what trips this one.
    assert verdict.saturated is True
    assert verdict.recovery_verdict.saturated is True


def test_outage_window_reported_saturated_raises_instead_of_being_treated_as_a_ramp_result():
    # A window that trips SATURATED despite outage_declared=True throughout can only mean
    # the outage-aware suppression itself is broken -- this must never surface as an
    # ordinary "level N saturated" ramp result.
    bad_outage_window = _observations(
        6,
        disk_buffer_bytes=lambda i: _GROWTH_STEP_BYTES * (i + 1),
        outage_declared=False,  # bug: not declared
    )
    cycle = DrainCycleResult(
        outage_length_s=30.0,
        connections=20,
        rate_hz=10.0,
        peak_buffer_bytes=600_000.0,
        drain_time_s=5.0,
        outage_window=bad_outage_window,
        recovery_window=_observations(6, disk_buffer_bytes=0.0, outage_declared=False),
    )

    with pytest.raises(OutageMisreportedError, match="outage-aware suppression"):
        evaluate_cycle(cycle)


def test_too_few_outage_samples_raises_rather_than_defaulting_to_healthy():
    cycle = DrainCycleResult(
        outage_length_s=5.0,
        connections=20,
        rate_hz=10.0,
        peak_buffer_bytes=1_000.0,
        drain_time_s=1.0,
        outage_window=_observations(2, disk_buffer_bytes=1_000.0, outage_declared=True),
        recovery_window=_observations(6, disk_buffer_bytes=0.0, outage_declared=False),
    )

    with pytest.raises(OutageMisreportedError, match="too few"):
        evaluate_cycle(cycle)


# --- find_drain_rate_curve / build_axis_run: fake-driver ramp, no I/O ----------------


def _fake_driver_draining_up_to(threshold_s: float):
    """A fake driver: cycles at an outage length below `threshold_s` drain in 5s; at or
    above it, the backlog never clears."""

    def driver(outage_length_s: float) -> DrainCycleResult:
        drains = outage_length_s < threshold_s
        return _growing_outage_cycle(outage_length_s, drain_time_s=5.0 if drains else None)

    return driver


def test_the_knee_is_the_shortest_outage_length_whose_backlog_never_drains():
    driver = _fake_driver_draining_up_to(threshold_s=100.0)

    axis_run, cycles = find_drain_rate_curve(
        driver, outage_lengths_s=[30.0, 60.0, 120.0], connections=20, rate_hz=10.0
    )

    assert axis_run.outcome == RunOutcome.KNEE_FOUND
    assert axis_run.tripped_level == 120.0
    assert axis_run.highest_clear_level == 60.0
    assert set(cycles) == {30.0, 60.0, 120.0}


def test_a_backlog_that_always_drains_reports_bound_not_found_never_a_fabricated_ceiling():
    driver = _fake_driver_draining_up_to(threshold_s=10_000.0)

    axis_run, _cycles = find_drain_rate_curve(
        driver, outage_lengths_s=[30.0, 60.0], connections=20, rate_hz=10.0
    )

    assert axis_run.outcome == RunOutcome.BOUND_NOT_FOUND
    assert axis_run.tripped_level is None
    assert axis_run.highest_clear_level == 60.0


def test_axis_run_uses_the_stable_curve_reporter_shape():
    driver = _fake_driver_draining_up_to(threshold_s=100.0)

    axis_run, _cycles = find_drain_rate_curve(
        driver, outage_lengths_s=[30.0, 60.0, 120.0], connections=20, rate_hz=10.0
    )

    assert axis_run.axis == "drain_rate"
    assert len(axis_run.points) == 3
    assert all(p.kind == PointKind.MEASURED for p in axis_run.points)
    assert all(p.composition.synthetic_senders == 20 for p in axis_run.points)
    assert all(p.composition.real_stacks == 0 for p in axis_run.points)
    saturated_levels = {p.level for p in axis_run.points if p.saturated}
    assert saturated_levels == {120.0}


def test_build_axis_run_records_the_tripped_level_as_the_only_saturated_point():
    ramp_result = RampResult(
        outcome=RampOutcome.KNEE_FOUND,
        highest_clear_level=60.0,
        tripped_level=120.0,
        tripped_verdict=None,
        levels_tried=(30.0, 60.0, 120.0),
    )
    cycles = {
        30.0: _growing_outage_cycle(30.0, drain_time_s=5.0),
        60.0: _growing_outage_cycle(60.0, drain_time_s=8.0),
        120.0: _growing_outage_cycle(120.0, drain_time_s=None),
    }

    axis_run = build_axis_run(ramp_result, cycles, connections=20, rate_hz=10.0)

    assert [p.level for p in axis_run.points] == [30.0, 60.0, 120.0]
    assert [p.saturated for p in axis_run.points] == [False, False, True]


def test_find_knee_stops_driving_once_a_cycle_fails_to_drain():
    driver = _fake_driver_draining_up_to(threshold_s=50.0)
    calls = []

    def counting_driver(outage_length_s: float) -> DrainCycleResult:
        calls.append(outage_length_s)
        return driver(outage_length_s)

    find_drain_rate_curve(
        counting_driver, outage_lengths_s=[30.0, 60.0, 120.0], connections=20, rate_hz=10.0
    )

    assert calls == [30.0, 60.0]


def test_find_knee_is_reused_unmodified_for_the_ramp():
    # Sanity check that find_drain_rate_curve is really delegating to
    # ramp_controller.find_knee() rather than reimplementing ramp semantics -- same
    # outcome, tripped/highest-clear levels, for the same driver/probe shape.
    driver = _fake_driver_draining_up_to(threshold_s=100.0)

    axis_run, cycles = find_drain_rate_curve(
        driver, outage_lengths_s=[30.0, 60.0, 120.0], connections=20, rate_hz=10.0
    )

    direct_result = find_knee(
        lambda level: cycles[level],
        lambda cycle: evaluate_cycle(cycle, SaturationBounds()),
        RampPolicy(levels=[30.0, 60.0, 120.0]),
    )
    assert direct_result.outcome.value == axis_run.outcome.value
    assert direct_result.tripped_level == axis_run.tripped_level
    assert direct_result.highest_clear_level == axis_run.highest_clear_level
