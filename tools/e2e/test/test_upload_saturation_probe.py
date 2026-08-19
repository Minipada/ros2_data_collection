# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for tools/e2e/scripts/upload_saturation_probe.py (#384): the Uploader
concurrency axis's saturation verdict, mirroring test_saturation_probe.py's coverage —
signal priority, trend-not-utilisation, outage suppression, and the insufficient-data
contract — for this axis's own two signals (queue depth, verify-then-delete backlog).
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

from upload_saturation_probe import (
    Observation,
    Signal,
    VerdictStatus,
    evaluate,
)

DEFAULT_QUEUE_DEPTH = 1.0
DEFAULT_BACKLOG = 0.0


def _window(
    queue_depths: list[float] | None = None,
    backlogs: list[float] | None = None,
    outages: list[bool] | None = None,
    length: int = 8,
) -> list[Observation]:
    """Builds a window from per-field lists, defaulting any omitted field to a flat,
    healthy series so a test only has to state the field(s) it cares about."""
    queue_depths = queue_depths if queue_depths is not None else [DEFAULT_QUEUE_DEPTH] * length
    backlogs = backlogs if backlogs is not None else [DEFAULT_BACKLOG] * length
    n = len(queue_depths)
    outages = outages if outages is not None else [False] * n
    assert len(backlogs) == len(outages) == n
    return [
        Observation(
            queue_depth=queue_depths[i],
            verify_delete_backlog=backlogs[i],
            outage_declared=outages[i],
        )
        for i in range(n)
    ]


def test_a_steady_healthy_window_returns_not_saturated():
    verdict = evaluate(_window())
    assert verdict.status == VerdictStatus.NOT_SATURATED
    assert verdict.saturated is False
    assert verdict.binding_signal is None


def test_a_monotonic_queue_depth_climb_returns_saturated_with_queue_depth_named():
    depths = [1.0, 1.0, 2.0, 4.0, 6.0, 9.0]
    verdict = evaluate(_window(queue_depths=depths, length=len(depths)))
    assert verdict.status == VerdictStatus.SATURATED
    assert verdict.binding_signal == Signal.QUEUE_DEPTH
    assert verdict.figures["queue_depth_growth"] >= 2.0


def test_a_flat_queue_depth_does_not_trip_the_verdict():
    depths = [2.0, 3.0, 2.0, 3.0, 2.0, 3.0]
    verdict = evaluate(_window(queue_depths=depths, length=len(depths)))
    assert verdict.status == VerdictStatus.NOT_SATURATED
    assert verdict.binding_signal is None


def test_a_high_but_flat_queue_depth_does_not_trip_the_verdict():
    # Depth measured by trend, not by a level threshold — a queue that is deep but not
    # growing is not this axis's saturation signal, same rejection of utilisation-alone
    # saturation_probe.py makes for the disk buffer.
    verdict = evaluate(_window(queue_depths=[500.0] * 8))
    assert verdict.status == VerdictStatus.NOT_SATURATED


def test_verify_delete_backlog_growth_trips_the_verdict_as_the_secondary_signal():
    backlogs = [0.0, 0.0, 1.0, 3.0, 5.0, 8.0]
    verdict = evaluate(_window(backlogs=backlogs, length=len(backlogs)))
    assert verdict.status == VerdictStatus.SATURATED
    assert verdict.binding_signal == Signal.VERIFY_DELETE_BACKLOG


def test_a_flat_verify_delete_backlog_does_not_trip_the_verdict():
    backlogs = [1.0, 0.0, 1.0, 0.0, 1.0, 0.0]
    verdict = evaluate(_window(backlogs=backlogs, length=len(backlogs)))
    assert verdict.status == VerdictStatus.NOT_SATURATED


def test_identical_backlog_growth_during_a_declared_outage_does_not_trip_the_verdict():
    backlogs = [0.0, 0.0, 1.0, 3.0, 5.0, 8.0]
    outages = [True] * len(backlogs)
    verdict = evaluate(_window(backlogs=backlogs, outages=outages, length=len(backlogs)))
    assert verdict.status == VerdictStatus.NOT_SATURATED
    assert verdict.binding_signal is None


def test_queue_depth_takes_priority_over_a_concurrently_growing_backlog():
    depths = [1.0, 1.0, 2.0, 4.0, 6.0, 9.0]
    backlogs = [0.0, 0.0, 1.0, 3.0, 5.0, 8.0]
    verdict = evaluate(_window(queue_depths=depths, backlogs=backlogs, length=len(depths)))
    assert verdict.binding_signal == Signal.QUEUE_DEPTH


def test_an_observation_window_with_insufficient_data_returns_an_explicit_result():
    verdict = evaluate(_window(length=2))
    assert verdict.status == VerdictStatus.INSUFFICIENT_DATA
    assert verdict.saturated is False
    assert verdict.binding_signal is None


def test_insufficient_data_is_returned_even_when_the_few_samples_look_saturated():
    verdict = evaluate(_window(queue_depths=[50.0, 90.0], length=2))
    assert verdict.status == VerdictStatus.INSUFFICIENT_DATA


def test_an_empty_window_returns_insufficient_data_not_an_error():
    verdict = evaluate([])
    assert verdict.status == VerdictStatus.INSUFFICIENT_DATA
