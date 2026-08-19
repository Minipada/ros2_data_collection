# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for tools/e2e/scripts/ramp_controller.py (#379): the acceptance criteria
from the issue, driven entirely with fake drivers/probes (no I/O, no saturation_probe
import needed — the controller is axis- and probe-agnostic per #323's PRD).
"""

import os
import sys
from dataclasses import dataclass

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

from ramp_controller import (
    RampOutcome,
    RampPolicy,
    find_knee,
)


@dataclass(frozen=True)
class FakeVerdict:
    saturated: bool


def _driver_and_probe_saturating_at(threshold: float):
    """A fake driver/probe pair: the driver's "state" for a level is just the level
    itself, and the probe reports saturated once the level reaches threshold."""

    def driver(level: float) -> float:
        return level

    def probe(state: float) -> FakeVerdict:
        return FakeVerdict(saturated=state >= threshold)

    return driver, probe


def test_a_driver_that_saturates_at_a_known_level_reports_that_level_as_tripped():
    driver, probe = _driver_and_probe_saturating_at(30.0)
    policy = RampPolicy(levels=[10.0, 20.0, 30.0, 40.0])

    result = find_knee(driver, probe, policy)

    assert result.outcome == RampOutcome.KNEE_FOUND
    assert result.tripped_level == 30.0
    assert result.highest_clear_level == 20.0
    assert result.tripped_verdict.saturated is True


def test_a_driver_that_never_saturates_within_ramp_bounds_reports_bound_not_found():
    driver, probe = _driver_and_probe_saturating_at(1_000.0)
    policy = RampPolicy(levels=[10.0, 20.0, 30.0])

    result = find_knee(driver, probe, policy)

    assert result.outcome == RampOutcome.BOUND_NOT_FOUND
    assert result.tripped_level is None
    assert result.tripped_verdict is None
    # The highest level tried is informational, not a fabricated ceiling — the
    # outcome above is what a caller must check before treating it as a bound.
    assert result.highest_clear_level == 30.0


def test_a_driver_that_saturates_immediately_at_the_lowest_level_reports_that_level():
    driver, probe = _driver_and_probe_saturating_at(10.0)
    policy = RampPolicy(levels=[10.0, 20.0, 30.0])

    result = find_knee(driver, probe, policy)

    assert result.outcome == RampOutcome.KNEE_FOUND
    assert result.tripped_level == 10.0
    assert result.highest_clear_level is None


def test_the_controller_stops_driving_load_once_saturation_trips():
    calls = []

    def driver(level: float) -> float:
        calls.append(level)
        return level

    def probe(state: float) -> FakeVerdict:
        return FakeVerdict(saturated=state >= 20.0)

    policy = RampPolicy(levels=[10.0, 20.0, 30.0, 40.0])

    find_knee(driver, probe, policy)

    assert calls == [10.0, 20.0]


# --- Ramp-policy boundary cases ---------------------------------------------------


def test_boundary_first_step_saturating_reports_no_clear_level_below_it():
    driver, probe = _driver_and_probe_saturating_at(1.0)
    policy = RampPolicy(levels=[1.0, 2.0, 3.0])

    result = find_knee(driver, probe, policy)

    assert result.tripped_level == 1.0
    assert result.highest_clear_level is None


def test_boundary_last_step_saturating_reports_every_earlier_level_as_clear():
    driver, probe = _driver_and_probe_saturating_at(3.0)
    policy = RampPolicy(levels=[1.0, 2.0, 3.0])

    result = find_knee(driver, probe, policy)

    assert result.tripped_level == 3.0
    assert result.highest_clear_level == 2.0
    assert result.levels_tried == (1.0, 2.0, 3.0)


def test_boundary_single_step_ramp_that_saturates_reports_that_step_tripped():
    driver, probe = _driver_and_probe_saturating_at(5.0)
    policy = RampPolicy(levels=[5.0])

    result = find_knee(driver, probe, policy)

    assert result.outcome == RampOutcome.KNEE_FOUND
    assert result.tripped_level == 5.0
    assert result.highest_clear_level is None


def test_boundary_single_step_ramp_that_stays_clear_reports_bound_not_found():
    driver, probe = _driver_and_probe_saturating_at(50.0)
    policy = RampPolicy(levels=[5.0])

    result = find_knee(driver, probe, policy)

    assert result.outcome == RampOutcome.BOUND_NOT_FOUND
    assert result.highest_clear_level == 5.0
    assert result.tripped_level is None


def test_an_empty_ramp_policy_is_rejected():
    with pytest.raises(ValueError, match="at least one level"):
        RampPolicy(levels=[])
