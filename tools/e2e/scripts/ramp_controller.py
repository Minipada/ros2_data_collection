# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Ramp controller (#379): drives load upward for #323's limits harness, one axis at a
time, until the saturation probe trips.

Given a driver, a probe, and a ramp policy, `find_knee()` walks the policy's levels in
ascending order — driving load at each one and asking the probe whether it saturated —
and reports the knee: the highest level whose verdict stayed clear, plus the level that
tripped it. It never drives past the first tripped level, and it never fabricates a
ceiling: a policy whose every level stays clear is reported as `BOUND_NOT_FOUND`, not as
a knee at the top of the ramp.

The controller touches no I/O itself and knows nothing about what a "level" means (a
connection count, a rate, an upload concurrency, …) — that's entirely the injected
driver's concern, per #323's PRD note that the ramp controller and the saturation probe
are axis-agnostic. `driver` and `probe` are plain callables, so every case here is
covered with in-process fakes.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Protocol


class RampOutcome(StrEnum):
    KNEE_FOUND = "knee_found"
    BOUND_NOT_FOUND = "bound_not_found"


class SaturationVerdict(Protocol):
    """The minimal shape find_knee() needs from a probe's return value — matches
    saturation_probe.Verdict's `saturated` property without importing that module, so a
    fake probe in a test needs nothing but this one attribute."""

    @property
    def saturated(self) -> bool: ...


@dataclass(frozen=True)
class RampPolicy:
    """The ascending sequence of load levels to try. Levels are compared only for
    identity in the result (never reordered), so callers are responsible for supplying
    them in ascending order."""

    levels: Sequence[float]

    def __post_init__(self) -> None:
        if len(self.levels) == 0:
            raise ValueError("a ramp policy must declare at least one level")


@dataclass(frozen=True)
class RampResult:
    outcome: RampOutcome
    # The highest level tried whose verdict stayed clear. None only when the very
    # first level tried already saturated.
    highest_clear_level: float | None
    # The level whose verdict tripped the probe. None when outcome is
    # BOUND_NOT_FOUND — never fabricated as a ceiling.
    tripped_level: float | None
    tripped_verdict: SaturationVerdict | None
    levels_tried: tuple[float, ...] = field(default_factory=tuple)


Driver = Callable[[float], object]
Probe = Callable[[object], SaturationVerdict]


def find_knee(driver: Driver, probe: Probe, policy: RampPolicy) -> RampResult:
    """Drives `driver(level)` for each level in `policy.levels`, ascending, evaluating
    `probe()` on the result after each one. Stops at the first saturated verdict."""
    highest_clear: float | None = None
    tried: list[float] = []

    for level in policy.levels:
        tried.append(level)
        verdict = probe(driver(level))
        if verdict.saturated:
            return RampResult(
                outcome=RampOutcome.KNEE_FOUND,
                highest_clear_level=highest_clear,
                tripped_level=level,
                tripped_verdict=verdict,
                levels_tried=tuple(tried),
            )
        highest_clear = level

    return RampResult(
        outcome=RampOutcome.BOUND_NOT_FOUND,
        highest_clear_level=highest_clear,
        tripped_level=None,
        tripped_verdict=None,
        levels_tried=tuple(tried),
    )
