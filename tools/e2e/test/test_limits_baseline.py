# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for tools/e2e/scripts/limits_baseline.py (#386): the one-command entry
point's combining logic, driven entirely with synthetic per-axis reports (no
containers, no dependency on the axis scripts actually running)."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

from curve_reporter import (
    AxisRun,
    LevelResult,
    PointKind,
    ResourceSample,
    RunComposition,
    RunOutcome,
    build_report,
)
from limits_baseline import merge_reports, render_baseline


def _measured(level: float, saturated: bool, real: int = 1, synthetic: int = 0) -> LevelResult:
    return LevelResult(
        level=level,
        saturated=saturated,
        kind=PointKind.MEASURED,
        composition=RunComposition(real_stacks=real, synthetic_senders=synthetic),
        resources=ResourceSample(cpu_percent=50.0, mem_percent=40.0, disk_percent=10.0),
    )


def _knee_found_report(axis: str, unit: str = "robots") -> object:
    run = AxisRun(
        axis=axis,
        unit=unit,
        outcome=RunOutcome.KNEE_FOUND,
        highest_clear_level=10.0,
        tripped_level=20.0,
        points=[
            _measured(10.0, False, real=1, synthetic=9),
            _measured(20.0, True, real=1, synthetic=19),
        ],
    )
    return build_report([run])


# --- Acceptance criterion: one combined artifact, stable/diffable ------------------


def test_merge_combines_all_axes_from_all_reports():
    reports = [
        _knee_found_report("shipper_fan_in_unconstrained"),
        _knee_found_report("single_robot_ceiling", unit="records_s"),
        _knee_found_report("uploader_concurrency", unit="uploads"),
        _knee_found_report("drain_rate", unit="seconds"),
    ]

    combined = merge_reports(reports)

    assert [a.axis for a in combined.axes] == [
        "drain_rate",
        "shipper_fan_in_unconstrained",
        "single_robot_ceiling",
        "uploader_concurrency",
    ]


def test_merge_is_stable_regardless_of_input_order():
    a = _knee_found_report("shipper_fan_in_unconstrained")
    b = _knee_found_report("single_robot_ceiling", unit="records_s")
    c = _knee_found_report("drain_rate", unit="seconds")

    forward = merge_reports([a, b, c])
    shuffled = merge_reports([c, a, b])

    assert forward == shuffled


def test_merge_rejects_duplicate_axis_names():
    reports = [
        _knee_found_report("shipper_fan_in_unconstrained"),
        _knee_found_report("shipper_fan_in_unconstrained"),
    ]

    with pytest.raises(ValueError, match="duplicate axis name"):
        merge_reports(reports)


# --- Acceptance criterion: each axis's closing sentence is present in the output ---


def test_render_baseline_includes_every_axis_closing_sentence():
    combined = merge_reports(
        [
            _knee_found_report("shipper_fan_in_unconstrained"),
            _knee_found_report("single_robot_ceiling", unit="records_s"),
            _knee_found_report("uploader_concurrency", unit="uploads"),
            _knee_found_report("drain_rate", unit="seconds"),
        ]
    )

    rendered = render_baseline(combined)

    assert "Closing statements:" in rendered
    assert "- shipper_fan_in_unconstrained sustains 10.0 robots" in rendered
    assert "- single_robot_ceiling sustains 10.0 records_s" in rendered
    assert "- uploader_concurrency sustains 10.0 uploads" in rendered
    assert "- drain_rate sustains 10.0 seconds" in rendered


def test_render_baseline_states_composition_and_measured_condition():
    combined = merge_reports([_knee_found_report("shipper_fan_in_unconstrained")])

    rendered = render_baseline(combined)

    assert "1 real + 9 synthetic" in rendered
    assert "not extrapolated" in rendered
