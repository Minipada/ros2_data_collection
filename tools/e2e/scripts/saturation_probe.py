# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Saturation probe (#377): a pure verdict function for #323's limits harness. Given a
chronologically ordered window of observations, decides whether the pipeline has
saturated and, if so, names the single binding signal responsible.

Signals are checked in the PRD's stated priority order, each only once every
higher-priority signal has failed to trip:

1. **Acknowledgement latency** crossing `ack_latency_bound_s` and staying there — the
   Shipper acknowledges a Record only once it is durably buffered, so a sustained rise in
   that latency is the pipeline itself reporting it cannot keep up. "Staying there" is
   checked over the window's trailing `ack_latency_sustain_samples` observations, not the
   whole window, so a transient spike that has already recovered by the end of the
   window does not trip the verdict.
2. **Unacked-window depth** trending upward — checked only once latency hasn't already
   decided the verdict, since a growing window is latency's own leading indicator, not an
   independent failure mode.
3. **Disk-buffer growth at steady state** — checked last, and only when no observation in
   the window has `outage_declared` set. Buffer *utilisation* alone is deliberately never
   read as a signal (see #323's PRD): a full buffer during a declared outage is the
   buffer working correctly, and the same growth with no outage declared means the
   Shipper is undersized. The two are told apart by the outage flag and the growth
   trend, never by a threshold on the level itself.

An observation window shorter than `min_observations` returns `INSUFFICIENT_DATA` rather
than any healthy/saturated verdict — a probe run before enough data has accumulated must
never be misread as "not saturated" by default.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from statistics import fmean


class VerdictStatus(StrEnum):
    NOT_SATURATED = "not_saturated"
    SATURATED = "saturated"
    INSUFFICIENT_DATA = "insufficient_data"


class Signal(StrEnum):
    ACK_LATENCY = "ack_latency"
    UNACKED_WINDOW_DEPTH = "unacked_window_depth"
    DISK_BUFFER = "disk_buffer"


@dataclass(frozen=True)
class Observation:
    """One sample in the window. Every check below assumes oldest-to-newest order."""

    ack_latency_s: float
    unacked_window_depth: float
    disk_buffer_bytes: float
    outage_declared: bool = False


@dataclass(frozen=True)
class SaturationBounds:
    # Fewer than this many observations can't support a trend judgement at all.
    min_observations: int = 4
    # The stated acknowledgement-latency percentile bound (the PRD's primary signal),
    # e.g. a p95 ack-latency SLO of 2s. Each Observation's ack_latency_s is assumed to
    # already be that percentile figure for its sample interval — this module doesn't
    # compute percentiles itself, only whether the figure crosses the bound and stays.
    ack_latency_bound_s: float = 2.0
    # How many trailing observations must all sit at/above the bound for a crossing to
    # count as "staying there" rather than a transient spike.
    ack_latency_sustain_samples: int = 3
    # Minimum growth (mean of the window's second half minus its first half) in unacked
    # records to count as "trending upward".
    unacked_window_depth_growth_threshold: float = 20.0
    # Minimum growth (same head/tail split) in disk-buffer bytes to count as growth at
    # steady state.
    disk_buffer_growth_threshold_bytes: float = 1_000_000.0


@dataclass(frozen=True)
class Verdict:
    status: VerdictStatus
    binding_signal: Signal | None
    reason: str
    figures: dict[str, float] = field(default_factory=dict)

    @property
    def saturated(self) -> bool:
        return self.status == VerdictStatus.SATURATED


def evaluate(
    window: Sequence[Observation], bounds: SaturationBounds = SaturationBounds()
) -> Verdict:
    """The saturation verdict for one observation window. Never raises on a short or
    empty window — that's INSUFFICIENT_DATA, not an error, since a limits-harness ramp
    controller calls this on every poll, including before the window has filled."""
    if len(window) < bounds.min_observations:
        return Verdict(
            status=VerdictStatus.INSUFFICIENT_DATA,
            binding_signal=None,
            reason=(
                f"window has {len(window)} observation(s), fewer than the "
                f"{bounds.min_observations} required to assess a trend"
            ),
        )

    for signal_check in (_ack_latency_signal, _unacked_window_depth_signal, _disk_buffer_signal):
        verdict = signal_check(window, bounds)
        if verdict is not None:
            return verdict

    return Verdict(
        status=VerdictStatus.NOT_SATURATED,
        binding_signal=None,
        reason="no signal crossed its bound",
    )


def _split_halves(
    window: Sequence[Observation],
) -> tuple[Sequence[Observation], Sequence[Observation]]:
    midpoint = len(window) // 2
    return window[:midpoint], window[midpoint:]


def _ack_latency_signal(window: Sequence[Observation], bounds: SaturationBounds) -> Verdict | None:
    tail = window[-bounds.ack_latency_sustain_samples :]
    if not tail or any(o.ack_latency_s < bounds.ack_latency_bound_s for o in tail):
        return None
    return Verdict(
        status=VerdictStatus.SATURATED,
        binding_signal=Signal.ACK_LATENCY,
        reason=(
            f"acknowledgement latency has stayed at or above the "
            f"{bounds.ack_latency_bound_s}s bound for the trailing {len(tail)} "
            f"observation(s)"
        ),
        figures={
            "ack_latency_bound_s": bounds.ack_latency_bound_s,
            "ack_latency_tail_min_s": min(o.ack_latency_s for o in tail),
            "ack_latency_tail_max_s": max(o.ack_latency_s for o in tail),
        },
    )


def _unacked_window_depth_signal(
    window: Sequence[Observation], bounds: SaturationBounds
) -> Verdict | None:
    head, tail = _split_halves(window)
    if not head or not tail:
        return None
    head_mean = fmean(o.unacked_window_depth for o in head)
    tail_mean = fmean(o.unacked_window_depth for o in tail)
    growth = tail_mean - head_mean
    if growth < bounds.unacked_window_depth_growth_threshold:
        return None
    return Verdict(
        status=VerdictStatus.SATURATED,
        binding_signal=Signal.UNACKED_WINDOW_DEPTH,
        reason=(
            f"unacked-window depth grew by {growth:.1f} record(s) from the window's "
            f"first half to its second, at or above the "
            f"{bounds.unacked_window_depth_growth_threshold} bound"
        ),
        figures={
            "unacked_window_depth_head_mean": head_mean,
            "unacked_window_depth_tail_mean": tail_mean,
            "unacked_window_depth_growth": growth,
        },
    )


def _disk_buffer_signal(window: Sequence[Observation], bounds: SaturationBounds) -> Verdict | None:
    # A full/growing buffer during a declared outage is the buffer doing its job, not a
    # saturation signal — see the module docstring. Suppressed for the whole window, not
    # just the tail: a growth trend that started under an outage and is still unwinding
    # isn't steady-state growth either.
    if any(o.outage_declared for o in window):
        return None
    head, tail = _split_halves(window)
    if not head or not tail:
        return None
    head_mean = fmean(o.disk_buffer_bytes for o in head)
    tail_mean = fmean(o.disk_buffer_bytes for o in tail)
    growth = tail_mean - head_mean
    if growth < bounds.disk_buffer_growth_threshold_bytes:
        return None
    return Verdict(
        status=VerdictStatus.SATURATED,
        binding_signal=Signal.DISK_BUFFER,
        reason=(
            f"disk-buffer bytes grew by {growth:.0f} at steady state (no outage "
            f"declared in the window), at or above the "
            f"{bounds.disk_buffer_growth_threshold_bytes} bound"
        ),
        figures={
            "disk_buffer_head_mean": head_mean,
            "disk_buffer_tail_mean": tail_mean,
            "disk_buffer_growth": growth,
        },
    )
