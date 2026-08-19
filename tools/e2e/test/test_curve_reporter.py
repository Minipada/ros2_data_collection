# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for tools/e2e/scripts/curve_reporter.py (#380): the issue's acceptance
criteria, driven entirely with synthetic run data (no containers, no network, no
dependency on ramp_controller/saturation_probe)."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

from curve_reporter import (
    AxisRun,
    BindingConstraint,
    LevelResult,
    PointKind,
    ResourceKind,
    ResourceSample,
    RunComposition,
    RunOutcome,
    build_report,
    render_summary,
    to_json,
)


def _measured(
    level: float,
    saturated: bool,
    cpu: float,
    mem: float,
    disk: float,
    real: int = 1,
    synthetic: int = 0,
) -> LevelResult:
    return LevelResult(
        level=level,
        saturated=saturated,
        kind=PointKind.MEASURED,
        composition=RunComposition(real_stacks=real, synthetic_senders=synthetic),
        resources=ResourceSample(cpu_percent=cpu, mem_percent=mem, disk_percent=disk),
    )


def _extrapolated(level: float, saturated: bool, real: int = 1, synthetic: int = 0) -> LevelResult:
    return LevelResult(
        level=level,
        saturated=saturated,
        kind=PointKind.EXTRAPOLATED,
        composition=RunComposition(real_stacks=real, synthetic_senders=synthetic),
        resources=None,
    )


def _cpu_bound_run(axis: str = "shipper_fan_in") -> AxisRun:
    return AxisRun(
        axis=axis,
        unit="robots",
        outcome=RunOutcome.KNEE_FOUND,
        highest_clear_level=140.0,
        tripped_level=220.0,
        points=[
            _measured(60.0, False, cpu=41.0, mem=30.0, disk=12.0, real=2, synthetic=58),
            _measured(140.0, False, cpu=68.0, mem=44.0, disk=20.0, real=3, synthetic=137),
            _measured(220.0, True, cpu=94.2, mem=61.0, disk=33.0, real=4, synthetic=216),
        ],
    )


# --- Acceptance criterion: identical input -> identical (stable, diffable) output --


def test_identical_input_produces_identical_report_and_json():
    runs = [_cpu_bound_run()]

    report_a = build_report(runs)
    report_b = build_report(runs)

    assert report_a == report_b
    assert to_json(report_a) == to_json(report_b)


def test_axis_order_in_output_is_stable_regardless_of_input_order():
    run_a = _cpu_bound_run(axis="shipper_fan_in")
    run_b = _cpu_bound_run(axis="single_robot_ceiling")
    run_c = _cpu_bound_run(axis="drain_rate")

    report_forward = build_report([run_a, run_b, run_c])
    report_shuffled = build_report([run_c, run_a, run_b])

    assert to_json(report_forward) == to_json(report_shuffled)
    assert [a.axis for a in report_forward.axes] == [
        "drain_rate",
        "shipper_fan_in",
        "single_robot_ceiling",
    ]


def test_points_within_an_axis_are_reported_in_ascending_level_order():
    run = AxisRun(
        axis="uploader_concurrency",
        unit="uploads",
        outcome=RunOutcome.BOUND_NOT_FOUND,
        highest_clear_level=30.0,
        tripped_level=None,
        points=[
            _measured(30.0, False, cpu=20.0, mem=10.0, disk=5.0),
            _measured(10.0, False, cpu=15.0, mem=8.0, disk=4.0),
            _measured(20.0, False, cpu=18.0, mem=9.0, disk=4.5),
        ],
    )

    report = build_report([run])

    assert [p.level for p in report.axes[0].points] == [10.0, 20.0, 30.0]


# --- Acceptance criterion: binding constraint correctly identified -----------------


def test_binding_constraint_identifies_cpu_when_cpu_peaks_highest():
    report = build_report([_cpu_bound_run()])

    bc = report.axes[0].binding_constraint

    assert bc.resource == ResourceKind.CPU
    assert bc.percent == pytest.approx(94.2)


def test_binding_constraint_identifies_memory_when_memory_peaks_highest():
    run = AxisRun(
        axis="drain_rate",
        unit="minutes",
        outcome=RunOutcome.KNEE_FOUND,
        highest_clear_level=5.0,
        tripped_level=10.0,
        points=[
            _measured(5.0, False, cpu=30.0, mem=40.0, disk=10.0),
            _measured(10.0, True, cpu=55.0, mem=91.5, disk=20.0),
        ],
    )

    report = build_report([run])

    bc = report.axes[0].binding_constraint
    assert bc.resource == ResourceKind.MEMORY
    assert bc.percent == pytest.approx(91.5)


def test_binding_constraint_identifies_disk_when_disk_peaks_highest():
    run = AxisRun(
        axis="uploader_concurrency",
        unit="uploads",
        outcome=RunOutcome.KNEE_FOUND,
        highest_clear_level=8.0,
        tripped_level=16.0,
        points=[
            _measured(8.0, False, cpu=20.0, mem=25.0, disk=40.0),
            _measured(16.0, True, cpu=45.0, mem=50.0, disk=97.0),
        ],
    )

    report = build_report([run])

    bc = report.axes[0].binding_constraint
    assert bc.resource == ResourceKind.DISK
    assert bc.percent == pytest.approx(97.0)


def test_binding_constraint_tie_break_prefers_cpu_over_memory_over_disk():
    run = AxisRun(
        axis="shipper_fan_in",
        unit="robots",
        outcome=RunOutcome.KNEE_FOUND,
        highest_clear_level=None,
        tripped_level=1.0,
        points=[_measured(1.0, True, cpu=90.0, mem=90.0, disk=90.0)],
    )

    report = build_report([run])

    assert report.axes[0].binding_constraint.resource == ResourceKind.CPU


def test_binding_constraint_is_none_when_bound_not_found():
    run = AxisRun(
        axis="single_robot_ceiling",
        unit="records_s",
        outcome=RunOutcome.BOUND_NOT_FOUND,
        highest_clear_level=500.0,
        tripped_level=None,
        points=[_measured(500.0, False, cpu=30.0, mem=20.0, disk=10.0)],
    )

    report = build_report([run])

    bc = report.axes[0].binding_constraint
    assert bc == BindingConstraint(
        resource=None,
        percent=None,
        reason="no saturation observed within tested range; no binding constraint to report",
    )


def test_binding_constraint_is_none_when_knee_found_but_no_measured_resources():
    run = AxisRun(
        axis="drain_rate",
        unit="minutes",
        outcome=RunOutcome.KNEE_FOUND,
        highest_clear_level=None,
        tripped_level=5.0,
        points=[_extrapolated(5.0, True)],
    )

    report = build_report([run])

    bc = report.axes[0].binding_constraint
    assert bc.resource is None
    assert "no measured resource sample" in bc.reason


# --- Acceptance criterion: measured vs extrapolated labelled honestly --------------


def test_extrapolated_points_never_carry_a_resource_sample_in_the_report():
    run = AxisRun(
        axis="shipper_fan_in",
        unit="robots",
        outcome=RunOutcome.KNEE_FOUND,
        highest_clear_level=140.0,
        tripped_level=220.0,
        points=[
            _measured(140.0, False, cpu=68.0, mem=44.0, disk=20.0),
            _measured(220.0, True, cpu=94.2, mem=61.0, disk=33.0),
            _extrapolated(500.0, True),
        ],
    )

    report = build_report([run])
    points_by_level = {p.level: p for p in report.axes[0].points}

    assert points_by_level[140.0].kind == PointKind.MEASURED
    assert points_by_level[140.0].resources is not None
    assert points_by_level[220.0].kind == PointKind.MEASURED
    assert points_by_level[500.0].kind == PointKind.EXTRAPOLATED
    assert points_by_level[500.0].resources is None


def test_human_summary_labels_each_point_measured_or_extrapolated():
    run = AxisRun(
        axis="shipper_fan_in",
        unit="robots",
        outcome=RunOutcome.KNEE_FOUND,
        highest_clear_level=140.0,
        tripped_level=220.0,
        points=[
            _measured(140.0, False, cpu=68.0, mem=44.0, disk=20.0),
            _measured(220.0, True, cpu=94.2, mem=61.0, disk=33.0),
            _extrapolated(500.0, True),
        ],
    )

    summary = render_summary(build_report([run]))

    assert "140.0 robots: clear [measured]" in summary
    assert "220.0 robots: saturated [measured]" in summary
    assert "500.0 robots: saturated [extrapolated]" in summary
    # The extrapolated point must never be printed with fabricated resource figures.
    assert "500.0 robots: saturated [extrapolated] (1 real + 0 synthetic) not measured" in summary


def test_human_summary_names_the_binding_constraint_at_saturation():
    summary = render_summary(build_report([_cpu_bound_run()]))

    assert "binding constraint: cpu at 94.2%" in summary


def test_human_summary_states_bound_not_found_without_a_fabricated_ceiling():
    run = AxisRun(
        axis="single_robot_ceiling",
        unit="records_s",
        outcome=RunOutcome.BOUND_NOT_FOUND,
        highest_clear_level=500.0,
        tripped_level=None,
        points=[_measured(500.0, False, cpu=30.0, mem=20.0, disk=10.0)],
    )

    summary = render_summary(build_report([run]))

    assert "bound not found within tested range (highest clear: 500.0)" in summary
    assert "binding constraint: none" in summary


# --- Composition (real vs synthetic) is reported per point -------------------------


def test_composition_is_reported_per_point():
    report = build_report([_cpu_bound_run()])

    points_by_level = {p.level: p for p in report.axes[0].points}
    assert points_by_level[60.0].composition == RunComposition(real_stacks=2, synthetic_senders=58)
    assert points_by_level[220.0].composition == RunComposition(
        real_stacks=4, synthetic_senders=216
    )


def test_composition_rejects_negative_counts():
    with pytest.raises(ValueError, match="cannot be negative"):
        RunComposition(real_stacks=-1, synthetic_senders=0)


# --- Structural validation -----------------------------------------------------------


def test_axis_run_rejects_empty_points():
    with pytest.raises(ValueError, match="at least one point"):
        AxisRun(
            axis="shipper_fan_in",
            unit="robots",
            outcome=RunOutcome.BOUND_NOT_FOUND,
            highest_clear_level=None,
            tripped_level=None,
            points=[],
        )


def test_axis_run_rejects_an_unnamed_axis():
    with pytest.raises(ValueError, match="name its axis"):
        AxisRun(
            axis="",
            unit="robots",
            outcome=RunOutcome.BOUND_NOT_FOUND,
            highest_clear_level=None,
            tripped_level=None,
            points=[_measured(1.0, False, cpu=1.0, mem=1.0, disk=1.0)],
        )


def test_resource_sample_rejects_negative_percentages():
    with pytest.raises(ValueError, match="cannot be negative"):
        ResourceSample(cpu_percent=-1.0, mem_percent=0.0, disk_percent=0.0)


# --- Multiple axes report independently ---------------------------------------------


def test_report_covers_multiple_axes_each_with_its_own_binding_constraint():
    cpu_run = _cpu_bound_run(axis="shipper_fan_in")
    disk_run = AxisRun(
        axis="uploader_concurrency",
        unit="uploads",
        outcome=RunOutcome.KNEE_FOUND,
        highest_clear_level=8.0,
        tripped_level=16.0,
        points=[
            _measured(8.0, False, cpu=20.0, mem=25.0, disk=40.0),
            _measured(16.0, True, cpu=45.0, mem=50.0, disk=97.0),
        ],
    )

    report = build_report([cpu_run, disk_run])

    by_axis = {a.axis: a for a in report.axes}
    assert by_axis["shipper_fan_in"].binding_constraint.resource == ResourceKind.CPU
    assert by_axis["uploader_concurrency"].binding_constraint.resource == ResourceKind.DISK
