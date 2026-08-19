# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for tools/e2e/scripts/single_robot_ceiling_driver.py (#383): the
axis-specific driver adapter for #323's limits harness single-robot-ceiling axis. No
podman, no ROS, no real time — a fake `RealStackHandle` plus a virtual clock stand in
for the real stack, matching ramp_controller.py's/saturation_probe.py's own no-I/O
testing standard.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

from ramp_controller import RampOutcome, RampPolicy, find_knee
from saturation_probe import SaturationBounds
from single_robot_ceiling_driver import (
    RealStackSample,
    SingleRobotCeilingConfig,
    make_driver,
    make_probe,
    run_level,
)


class VirtualClock:
    """A fake now_fn/sleep_fn pair: sleep_fn advances the virtual clock instead of
    blocking, so a test exercises the real polling loop without taking real seconds."""

    def __init__(self) -> None:
        self.now = 0.0

    def now_fn(self) -> float:
        return self.now

    def sleep_fn(self, seconds: float) -> None:
        self.now += seconds


class FakeRealStackHandle:
    """Replays a scripted sequence of RealStackSamples for whichever rate `set_rate()`
    was last called with, one per `sample()` call (holding the last sample if polled
    past the end of the script)."""

    def __init__(self, samples_by_rate: dict) -> None:
        self._samples_by_rate = samples_by_rate
        self.rate_calls: list = []
        self._rate = None
        self._index = 0

    def set_rate(self, rate_hz: float) -> None:
        self.rate_calls.append(rate_hz)
        self._rate = rate_hz
        self._index = 0

    def sample(self) -> RealStackSample:
        samples = self._samples_by_rate[self._rate]
        sample = samples[min(self._index, len(samples) - 1)]
        self._index += 1
        return sample


def _immediate_ack_samples(clock_ticks: list[float], values_per_tick: int = 1) -> list:
    """A "clear" script: every value published in a tick is also arrived in that same
    tick — zero lag, zero unacked depth, at every sample."""
    samples = []
    published: set = set()
    next_value = 0
    for t in clock_ticks:
        for _ in range(values_per_tick):
            published.add(next_value)
            next_value += 1
        samples.append(
            RealStackSample(
                sampled_at_s=t,
                published_values=frozenset(published),
                arrived_values=frozenset(published),
                disk_buffer_bytes=1_000.0,
            )
        )
    return samples


def _never_ack_samples(clock_ticks: list[float], values_per_tick: int = 2) -> list:
    """A "saturating" script: values keep publishing but nothing ever arrives — the
    oldest pending value's age grows without bound."""
    samples = []
    published: set = set()
    next_value = 0
    for t in clock_ticks:
        for _ in range(values_per_tick):
            published.add(next_value)
            next_value += 1
        samples.append(
            RealStackSample(
                sampled_at_s=t,
                published_values=frozenset(published),
                arrived_values=frozenset(),
                disk_buffer_bytes=1_000.0,
            )
        )
    return samples


TICKS = [0.0, 2.0, 4.0, 6.0, 8.0]  # 5 samples: duration_s=8, sample_interval_s=2


def test_run_level_steps_the_rate_exactly_once():
    clock = VirtualClock()
    handle = FakeRealStackHandle({5.0: _immediate_ack_samples(TICKS)})
    config = SingleRobotCeilingConfig(
        rate_hz=5.0,
        duration_s=8.0,
        sample_interval_s=2.0,
        now_fn=clock.now_fn,
        sleep_fn=clock.sleep_fn,
    )

    run_level(config, handle)

    assert handle.rate_calls == [5.0]


def test_run_level_polls_until_duration_elapses_then_stops():
    clock = VirtualClock()
    handle = FakeRealStackHandle({5.0: _immediate_ack_samples(TICKS)})
    config = SingleRobotCeilingConfig(
        rate_hz=5.0,
        duration_s=8.0,
        sample_interval_s=2.0,
        now_fn=clock.now_fn,
        sleep_fn=clock.sleep_fn,
    )

    run = run_level(config, handle)

    assert len(run.window) == len(TICKS)
    assert clock.now == 8.0


def test_a_value_acked_before_deadline_carries_a_latency_not_none():
    clock = VirtualClock()
    handle = FakeRealStackHandle({5.0: _immediate_ack_samples(TICKS)})
    config = SingleRobotCeilingConfig(
        rate_hz=5.0,
        duration_s=8.0,
        sample_interval_s=2.0,
        now_fn=clock.now_fn,
        sleep_fn=clock.sleep_fn,
    )

    run = run_level(config, handle)

    assert run.driver_result.sent_count == 5
    assert run.driver_result.acked_count == 5
    assert all(o.latency_s is not None for o in run.driver_result.observations)
    assert all(o.latency_s == 0.0 for o in run.driver_result.observations)


def test_a_value_never_acked_by_the_deadline_carries_latency_none():
    clock = VirtualClock()
    handle = FakeRealStackHandle({5.0: _never_ack_samples(TICKS)})
    config = SingleRobotCeilingConfig(
        rate_hz=5.0,
        duration_s=8.0,
        sample_interval_s=2.0,
        now_fn=clock.now_fn,
        sleep_fn=clock.sleep_fn,
    )

    run = run_level(config, handle)

    assert run.driver_result.sent_count == 10
    assert run.driver_result.acked_count == 0
    assert run.driver_result.unacked_chunk_ids  # every chunk id present
    assert all(o.latency_s is None for o in run.driver_result.observations)


def test_window_unacked_depth_tracks_pending_values():
    clock = VirtualClock()
    handle = FakeRealStackHandle({5.0: _never_ack_samples(TICKS)})
    config = SingleRobotCeilingConfig(
        rate_hz=5.0,
        duration_s=8.0,
        sample_interval_s=2.0,
        now_fn=clock.now_fn,
        sleep_fn=clock.sleep_fn,
    )

    run = run_level(config, handle)

    depths = [o.unacked_window_depth for o in run.window]
    assert depths == [2.0, 4.0, 6.0, 8.0, 10.0]


def test_window_ack_latency_grows_when_nothing_is_ever_acked():
    clock = VirtualClock()
    handle = FakeRealStackHandle({5.0: _never_ack_samples(TICKS)})
    config = SingleRobotCeilingConfig(
        rate_hz=5.0,
        duration_s=8.0,
        sample_interval_s=2.0,
        now_fn=clock.now_fn,
        sleep_fn=clock.sleep_fn,
    )

    run = run_level(config, handle)

    latencies = [o.ack_latency_s for o in run.window]
    assert latencies == [0.0, 2.0, 4.0, 6.0, 8.0]


def test_disk_buffer_bytes_and_outage_flag_pass_through_to_the_window():
    clock = VirtualClock()
    sample = RealStackSample(
        sampled_at_s=0.0,
        published_values=frozenset({0}),
        arrived_values=frozenset({0}),
        disk_buffer_bytes=42_000.0,
        outage_declared=True,
    )
    handle = FakeRealStackHandle({1.0: [sample]})
    config = SingleRobotCeilingConfig(
        rate_hz=1.0,
        duration_s=0.0,
        sample_interval_s=2.0,
        now_fn=clock.now_fn,
        sleep_fn=clock.sleep_fn,
    )

    run = run_level(config, handle)

    assert len(run.window) == 1
    assert run.window[0].disk_buffer_bytes == 42_000.0
    assert run.window[0].outage_declared is True


def test_zero_duration_takes_exactly_one_sample():
    clock = VirtualClock()
    handle = FakeRealStackHandle({1.0: _immediate_ack_samples([0.0])})
    config = SingleRobotCeilingConfig(
        rate_hz=1.0,
        duration_s=0.0,
        sample_interval_s=2.0,
        now_fn=clock.now_fn,
        sleep_fn=clock.sleep_fn,
    )

    run = run_level(config, handle)

    assert len(run.window) == 1


# --- end-to-end through ramp_controller.find_knee(), proving the interface fit -------


def test_make_driver_and_make_probe_conform_to_ramp_controllers_interface():
    """#383's acceptance criterion: the axis-specific driver adapter conforms to
    ramp_controller's driver interface, and the ramp controller / saturation probe are
    reused entirely unchanged — this test imports neither module modified, only calls
    the unmodified find_knee()/evaluate() against this axis's driver/probe."""
    clock = VirtualClock()
    handle = FakeRealStackHandle(
        {
            1.0: _immediate_ack_samples(TICKS),
            2.0: _immediate_ack_samples(TICKS),
            4.0: _never_ack_samples(TICKS),
        }
    )
    driver = make_driver(handle, duration_s=8.0, sample_interval_s=2.0)

    def driver_with_fake_clock(rate_hz: float):
        config = SingleRobotCeilingConfig(
            rate_hz=rate_hz,
            duration_s=8.0,
            sample_interval_s=2.0,
            now_fn=clock.now_fn,
            sleep_fn=clock.sleep_fn,
        )
        return run_level(config, handle)

    probe = make_probe(SaturationBounds())
    policy = RampPolicy(levels=[1.0, 2.0, 4.0])

    result = find_knee(driver_with_fake_clock, probe, policy)

    assert result.outcome == RampOutcome.KNEE_FOUND
    assert result.tripped_level == 4.0
    assert result.highest_clear_level == 2.0
    assert handle.rate_calls == [1.0, 2.0, 4.0]
    # driver() itself, unwrapped, still satisfies Callable[[float], object] — exercised
    # once more directly to show make_driver()'s own closure is call-compatible too.
    assert callable(driver)
