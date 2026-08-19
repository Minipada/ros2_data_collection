# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for tools/e2e/scripts/run_limits_single_robot_ceiling.py (#383): the pure
parsing helpers (canned command output -> value, no subprocess) and the
find_knee-result-to-AxisRun folding logic. Per #323's PRD ("The topology composer and
the scenario script are not unit-tested; running them is the test"), the real-I/O
`PodmanRealStackHandle` itself is exercised by actually running the scenario, not here.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

from curve_reporter import PointKind, ResourceSample, RunOutcome
from load_driver import LoadDriverResult
from ramp_controller import RampOutcome, RampResult
from run_limits_single_robot_ceiling import (
    build_axis_run,
    parse_arrived_values,
    parse_disk_bytes,
    parse_disk_percent,
    parse_podman_stats,
    parse_published_values,
)
from single_robot_ceiling_driver import SingleRobotCeilingRun


def test_parse_published_values_filters_by_topic_and_ignores_other_lines():
    ledger = "synth00,0\nsynth01,0\nsynth00,1\n#boundary,synth00,2\nsynth00,2\n"
    assert parse_published_values(ledger, "synth00") == frozenset({0, 1, 2})


def test_parse_published_values_empty_text():
    assert parse_published_values("", "synth00") == frozenset()


def test_parse_arrived_values_from_psql_rows():
    assert parse_arrived_values("0\n1\n2\n") == frozenset({0, 1, 2})


def test_parse_arrived_values_empty():
    assert parse_arrived_values("") == frozenset()


def test_parse_disk_bytes_from_du_output():
    assert parse_disk_bytes("12345\t/root/.dc/e2e/buffer\n") == 12345.0


def test_parse_disk_bytes_empty_is_zero():
    assert parse_disk_bytes("") == 0.0


def test_parse_podman_stats_percentages():
    assert parse_podman_stats("12.34%,56.78%\n") == (12.34, 56.78)


def test_parse_podman_stats_empty_is_zero():
    assert parse_podman_stats("") == (0.0, 0.0)


def test_parse_disk_percent_from_df_output():
    df_output = "Use%\n42%\n"
    assert parse_disk_percent(df_output) == 42.0


def test_parse_disk_percent_missing_value_line_is_zero():
    assert parse_disk_percent("Use%\n") == 0.0


def _fake_run(rate_hz: float) -> SingleRobotCeilingRun:
    return SingleRobotCeilingRun(
        rate_hz=rate_hz,
        driver_result=LoadDriverResult(),
        window=(),
    )


def test_build_axis_run_marks_only_the_tripped_level_saturated():
    result = RampResult(
        outcome=RampOutcome.KNEE_FOUND,
        highest_clear_level=20.0,
        tripped_level=40.0,
        tripped_verdict=None,
        levels_tried=(10.0, 20.0, 40.0),
    )
    recorded = [
        (10.0, _fake_run(10.0), ResourceSample(10.0, 10.0, 10.0)),
        (20.0, _fake_run(20.0), ResourceSample(20.0, 20.0, 20.0)),
        (40.0, _fake_run(40.0), ResourceSample(90.0, 30.0, 30.0)),
    ]

    axis_run = build_axis_run(result, recorded)

    assert axis_run.axis == "single_robot_ceiling"
    assert axis_run.outcome == RunOutcome.KNEE_FOUND
    assert axis_run.highest_clear_level == 20.0
    assert axis_run.tripped_level == 40.0
    saturated_levels = {p.level for p in axis_run.points if p.saturated}
    assert saturated_levels == {40.0}
    assert all(p.kind == PointKind.MEASURED for p in axis_run.points)
    assert all(p.composition.real_stacks == 1 for p in axis_run.points)
    assert all(p.composition.synthetic_senders == 0 for p in axis_run.points)


def test_build_axis_run_bound_not_found_marks_nothing_saturated():
    result = RampResult(
        outcome=RampOutcome.BOUND_NOT_FOUND,
        highest_clear_level=40.0,
        tripped_level=None,
        tripped_verdict=None,
        levels_tried=(10.0, 20.0, 40.0),
    )
    recorded = [
        (10.0, _fake_run(10.0), ResourceSample(10.0, 10.0, 10.0)),
        (20.0, _fake_run(20.0), ResourceSample(20.0, 20.0, 20.0)),
        (40.0, _fake_run(40.0), ResourceSample(30.0, 30.0, 30.0)),
    ]

    axis_run = build_axis_run(result, recorded)

    assert axis_run.outcome == RunOutcome.BOUND_NOT_FOUND
    assert axis_run.tripped_level is None
    assert all(not p.saturated for p in axis_run.points)
