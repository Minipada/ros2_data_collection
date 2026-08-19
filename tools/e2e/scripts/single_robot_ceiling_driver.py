# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Single-robot ceiling driver adapter (#383): the axis-specific driver for #323's
limits harness that ramps one real DC stack's Record emission rate — through its
Measurements, the Bridge, and its local Shipper — rather than the socket-level
synthetic sender #378's load_driver.py drives.

Per the issue, this axis "supplies its own axis-specific driver adapter — conforming to
the same target/rate/duration → ledger-plus-latency interface the synthetic driver
implements". `run_level()` plays that role: like `load_driver.run()`, it is handed a
rate and a duration and returns the same shape, `load_driver.LoadDriverResult` (a
sent-ledger plus per-entry acknowledgement-latency observations, `None` for one still
unacknowledged when the level ends) — but it gets there by polling the real stack
instead of holding open a raw TCP connection.

What "sent" and "acked" mean for the real path, since there is no per-frame ack visible
from outside dc_bridge's Forwarder/Shipper connection:

- **sent**: a synth-topic counter *value* the real `workload_generator.py` node has
  published, discovered by polling the value the ledger-checked topic has reached so
  far (a monotonic counter, so "new since last sample" is exact, unlike load_driver's
  ledger which is written record-by-record as it happens).
- **acked**: that same value having reached the destination Postgres table (`dc_records
  WHERE source = topic AND value = ...`) — the same natural key
  tools/e2e/scripts/verify_zero_loss.py already treats as ground truth. Its arrival
  time is only known to sampling resolution, so the reported latency is an
  upper-bound approximation of the real end-to-end delay, not the exact per-record
  figure load_driver.py's live TCP session gets — a discovery worth recording rather
  than presenting as more precise than it is.

The real stack's Shipper isn't the object of every observation, though: the harness
also samples its on-disk buffer directly (the same figure the saturation probe's third
signal, disk-buffer growth, needs) via the injected `RealStackHandle`, so this axis can
still trip on backpressure the destination-arrival lag alone wouldn't show.

`RealStackHandle` is the only real-I/O seam (podman exec, `ros2 param set`, a `psql`
query) — everything else here is pure and unit-tested with a fake handle, matching
ramp_controller.py/saturation_probe.py's own no-I/O testing standard. The topology
script (run_limits_single_robot_ceiling.sh / .py) supplies the real implementation.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from load_driver import AckObservation, LedgerEntry, LoadDriverResult
from saturation_probe import Observation, SaturationBounds, Verdict, evaluate


@dataclass(frozen=True)
class RealStackSample:
    """One poll of the real stack's state for the one synth topic this axis tracks.

    `published_values`/`arrived_values` are the *full* sets observed so far this level
    (not deltas) — cheap to recompute each poll (a ledger tail plus one `psql` query)
    and it means a dropped/late sample never desyncs the running totals.
    """

    sampled_at_s: float
    published_values: frozenset[int]
    arrived_values: frozenset[int]
    disk_buffer_bytes: float
    outage_declared: bool = False


class RealStackHandle(Protocol):
    """The two real-stack operations this driver needs. Injected so `run_level()` is
    unit-testable with a fake (no podman, no ROS, no network) — the topology script
    supplies the podman/`ros2 param set`/`psql`-backed implementation."""

    def set_rate(self, rate_hz: float) -> None: ...
    def sample(self) -> RealStackSample: ...


@dataclass(frozen=True)
class SingleRobotCeilingConfig:
    rate_hz: float
    duration_s: float
    sample_interval_s: float = 2.0
    tag: str = "single_robot_ceiling"
    now_fn: Callable[[], float] = time.monotonic
    sleep_fn: Callable[[float], None] = time.sleep


@dataclass(frozen=True)
class SingleRobotCeilingRun:
    """One ramp level's full result: the ledger-plus-latency shape #383's acceptance
    criterion calls for (`driver_result`), plus the windowed `saturation_probe.Observation`
    samples the injected probe evaluates (`window`) — this axis's `ramp_controller.Driver`
    return value."""

    rate_hz: float
    driver_result: LoadDriverResult
    window: tuple[Observation, ...]


def run_level(config: SingleRobotCeilingConfig, handle: RealStackHandle) -> SingleRobotCeilingRun:
    """Steps the real workload generator to `config.rate_hz` (`handle.set_rate()`),
    polls the real stack every `config.sample_interval_s` for `config.duration_s`, and
    returns both the ledger-plus-latency result and the observation window.

    Partially applying `handle` (see `make_driver()`) gives a callable matching
    `ramp_controller.Driver = Callable[[float], object]`.
    """
    handle.set_rate(config.rate_hz)

    first_seen_at: dict[int, float] = {}
    acked_at: dict[int, float] = {}
    window: list[Observation] = []

    deadline = config.now_fn() + config.duration_s
    while True:
        sample = handle.sample()
        for value in sample.published_values:
            first_seen_at.setdefault(value, sample.sampled_at_s)
        for value in sample.arrived_values & first_seen_at.keys():
            acked_at.setdefault(value, sample.sampled_at_s)

        pending_since = [first_seen_at[v] for v in first_seen_at if v not in acked_at]
        just_acked_latencies = [
            acked_at[v] - first_seen_at[v] for v in acked_at if acked_at[v] == sample.sampled_at_s
        ]
        if just_acked_latencies:
            ack_latency_s = max(just_acked_latencies)
        elif pending_since:
            ack_latency_s = sample.sampled_at_s - min(pending_since)
        else:
            ack_latency_s = 0.0

        window.append(
            Observation(
                ack_latency_s=ack_latency_s,
                unacked_window_depth=float(len(pending_since)),
                disk_buffer_bytes=sample.disk_buffer_bytes,
                outage_declared=sample.outage_declared,
            )
        )

        now = config.now_fn()
        if now >= deadline:
            break
        config.sleep_fn(min(config.sample_interval_s, deadline - now))

    ledger = [
        LedgerEntry(
            connection_id=0,
            seq=value,
            tag=config.tag,
            chunk_id=str(value),
            frame_bytes=0,
            sent_at=sent_at,
            payload={"value": value},
        )
        for value, sent_at in sorted(first_seen_at.items())
    ]
    observations = [
        AckObservation(
            connection_id=0,
            seq=value,
            chunk_id=str(value),
            latency_s=(acked_at[value] - sent_at) if value in acked_at else None,
        )
        for value, sent_at in sorted(first_seen_at.items())
    ]

    return SingleRobotCeilingRun(
        rate_hz=config.rate_hz,
        driver_result=LoadDriverResult(ledger=ledger, observations=observations),
        window=tuple(window),
    )


def make_driver(
    handle: RealStackHandle, duration_s: float, sample_interval_s: float = 2.0
) -> Callable[[float], SingleRobotCeilingRun]:
    """Builds the `ramp_controller.Driver`-conforming callable
    (`Callable[[float], object]`) `find_knee()` calls at each ramp level, closed over a
    real-stack handle and this axis's fixed per-level duration/polling cadence."""

    def driver(rate_hz: float) -> SingleRobotCeilingRun:
        config = SingleRobotCeilingConfig(
            rate_hz=rate_hz, duration_s=duration_s, sample_interval_s=sample_interval_s
        )
        return run_level(config, handle)

    return driver


def make_probe(
    bounds: SaturationBounds | None = None,
) -> Callable[[SingleRobotCeilingRun], Verdict]:
    """Builds the `ramp_controller.Probe`-conforming callable that hands each level's
    windowed observations to the *unmodified* `saturation_probe.evaluate()` — #383's
    acceptance criterion that the probe stays axis-agnostic; this module never
    reimplements or overrides its verdict logic."""
    resolved_bounds = bounds if bounds is not None else SaturationBounds()

    def probe(run: SingleRobotCeilingRun) -> Verdict:
        return evaluate(run.window, resolved_bounds)

    return probe
