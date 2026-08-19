# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for tools/e2e/scripts/saturation_probe.py (#377): the acceptance criteria
from the issue, plus the signal-priority ordering and the "buffer growth is told apart
from buffer utilisation" behaviour the PRD (#323) calls out explicitly.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

from saturation_probe import (
    Observation,
    Signal,
    VerdictStatus,
    evaluate,
)

DEFAULT_LATENCY = 0.2
DEFAULT_DEPTH = 5.0
DEFAULT_BUFFER = 1_000.0


def _window(
    latencies: list[float] | None = None,
    depths: list[float] | None = None,
    buffers: list[float] | None = None,
    outages: list[bool] | None = None,
    length: int = 8,
) -> list[Observation]:
    """Builds a window from per-field lists, defaulting any omitted field to a flat,
    healthy series so a test only has to state the field(s) it cares about."""
    latencies = latencies if latencies is not None else [DEFAULT_LATENCY] * length
    depths = depths if depths is not None else [DEFAULT_DEPTH] * length
    buffers = buffers if buffers is not None else [DEFAULT_BUFFER] * length
    n = len(latencies)
    outages = outages if outages is not None else [False] * n
    assert len(depths) == len(buffers) == len(outages) == n
    return [
        Observation(
            ack_latency_s=latencies[i],
            unacked_window_depth=depths[i],
            disk_buffer_bytes=buffers[i],
            outage_declared=outages[i],
        )
        for i in range(n)
    ]


def test_a_steady_healthy_window_returns_not_saturated():
    verdict = evaluate(_window())
    assert verdict.status == VerdictStatus.NOT_SATURATED
    assert verdict.saturated is False
    assert verdict.binding_signal is None


def test_a_monotonic_latency_climb_crossing_the_bound_returns_saturated_with_latency_named():
    latencies = [0.2, 0.3, 0.5, 1.0, 1.8, 2.2, 2.5, 2.8, 3.0, 3.0]
    verdict = evaluate(_window(latencies=latencies, length=len(latencies)))
    assert verdict.status == VerdictStatus.SATURATED
    assert verdict.binding_signal == Signal.ACK_LATENCY
    assert verdict.figures["ack_latency_tail_min_s"] >= 2.0


def test_a_transient_latency_spike_that_recovers_does_not_trip_the_verdict():
    latencies = [0.2, 0.2, 0.2, 3.0, 3.0, 0.2, 0.2, 0.2]
    verdict = evaluate(_window(latencies=latencies, length=len(latencies)))
    assert verdict.status == VerdictStatus.NOT_SATURATED
    assert verdict.binding_signal is None


def test_steady_state_disk_buffer_growth_with_no_outage_trips_the_verdict():
    buffers = [1_000.0, 1_000.0, 2_000.0, 3_000.0, 5_000_000.0, 8_000_000.0, 12_000_000.0]
    verdict = evaluate(_window(buffers=buffers, length=len(buffers)))
    assert verdict.status == VerdictStatus.SATURATED
    assert verdict.binding_signal == Signal.DISK_BUFFER


def test_identical_disk_buffer_growth_during_a_declared_outage_does_not_trip_the_verdict():
    buffers = [1_000.0, 1_000.0, 2_000.0, 3_000.0, 5_000_000.0, 8_000_000.0, 12_000_000.0]
    outages = [True] * len(buffers)
    verdict = evaluate(_window(buffers=buffers, outages=outages, length=len(buffers)))
    assert verdict.status == VerdictStatus.NOT_SATURATED
    assert verdict.binding_signal is None


def test_an_observation_window_with_insufficient_data_returns_an_explicit_result():
    verdict = evaluate(_window(length=2))
    assert verdict.status == VerdictStatus.INSUFFICIENT_DATA
    assert verdict.saturated is False
    assert verdict.binding_signal is None


def test_insufficient_data_is_returned_even_when_the_few_samples_look_saturated():
    # Every sample is already over the latency bound; a naive default would call this
    # healthy or saturated, but there aren't enough samples to say either.
    verdict = evaluate(_window(latencies=[3.0, 3.0], length=2))
    assert verdict.status == VerdictStatus.INSUFFICIENT_DATA


def test_an_empty_window_returns_insufficient_data_not_an_error():
    verdict = evaluate([])
    assert verdict.status == VerdictStatus.INSUFFICIENT_DATA


def test_unacked_window_depth_trending_upward_trips_the_verdict_as_the_secondary_signal():
    depths = [5.0, 5.0, 10.0, 20.0, 35.0, 55.0]
    verdict = evaluate(_window(depths=depths, length=len(depths)))
    assert verdict.status == VerdictStatus.SATURATED
    assert verdict.binding_signal == Signal.UNACKED_WINDOW_DEPTH


def test_a_flat_unacked_window_depth_does_not_trip_the_verdict():
    depths = [5.0, 6.0, 5.0, 6.0, 5.0, 6.0]
    verdict = evaluate(_window(depths=depths, length=len(depths)))
    assert verdict.status == VerdictStatus.NOT_SATURATED


def test_latency_takes_priority_over_a_concurrently_growing_unacked_window():
    latencies = [0.2, 0.3, 0.5, 1.0, 1.8, 2.2, 2.5, 2.8, 3.0, 3.0]
    depths = [5.0, 5.0, 10.0, 20.0, 35.0, 55.0, 80.0, 110.0, 145.0, 185.0]
    verdict = evaluate(_window(latencies=latencies, depths=depths, length=len(latencies)))
    assert verdict.binding_signal == Signal.ACK_LATENCY


def test_unacked_window_depth_takes_priority_over_a_concurrently_growing_disk_buffer():
    depths = [5.0, 5.0, 10.0, 20.0, 35.0, 55.0]
    buffers = [1_000.0, 1_000.0, 2_000.0, 3_000.0, 5_000_000.0, 8_000_000.0]
    verdict = evaluate(_window(depths=depths, buffers=buffers, length=len(depths)))
    assert verdict.binding_signal == Signal.UNACKED_WINDOW_DEPTH


def test_disk_buffer_growth_is_measured_by_trend_not_by_a_level_threshold():
    # A buffer that is high but flat (utilisation alone) must never trip the verdict —
    # only growth does, per the PRD's explicit rejection of utilisation as a signal.
    verdict = evaluate(_window(buffers=[50_000_000.0] * 8))
    assert verdict.status == VerdictStatus.NOT_SATURATED
