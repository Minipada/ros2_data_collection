# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Upload-concurrency saturation probe (#384): a pure verdict function for #323's
limits harness, sibling to saturation_probe.py but for the Uploader concurrency axis.

Files bypass the Shipper entirely — the Bridge's Uploader writes them to object storage
directly over its own AWS SDK client, with no Vector hop (#381's recorded discovery) —
so saturation_probe.py's Observation (ack latency, unacked-window depth, disk-buffer
bytes) doesn't apply here: there is no acknowledgement latency to sample, because there
is no acknowledging Shipper in this path. What the Uploader has instead is its own
backpressure statement, taken straight from dc_bridge/src/bridge_node.cpp's readiness
service and its own Postgres file_status/group_complete rows (ADR-0005):

1. **Upload queue depth** trending upward — `IntentQueue::size()`, sampled from the
   Bridge's own `~/ready` service (#265's "cheap observability hook"). The Uploader is a
   single synchronous thread per Bridge process; a queue that keeps growing across the
   window is that thread telling callers it cannot drain incoming Files as fast as they
   arrive, exactly the same "trend, not a snapshot" reasoning saturation_probe.py uses
   for unacked-window depth. The primary signal, since it is the axis's most direct
   backpressure statement (there is no Uploader-side acknowledgement latency to check
   first, unlike the Shipper's Record path).
2. **Verify-then-delete backlog** growing at steady state — the count of File rows that
   have verified (`uploaded=true`) but not yet cleared (`deleted=false`) in `dc_files`.
   Checked only once queue depth hasn't already decided the verdict, and suppressed for
   the whole window if `outage_declared` is set anywhere in it, mirroring
   saturation_probe.py's disk-buffer check: a growing backlog during a declared outage
   (the object-storage Destination is down) is expected, not a capacity signal; the same
   growth at steady state means delete-after-verify is falling behind the upload rate —
   this axis's namesake failure mode.

An observation window shorter than `min_observations` returns `INSUFFICIENT_DATA` rather
than any healthy/saturated verdict, same contract as saturation_probe.py.
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
    QUEUE_DEPTH = "queue_depth"
    VERIFY_DELETE_BACKLOG = "verify_delete_backlog"


@dataclass(frozen=True)
class Observation:
    """One sample in the window. Every check below assumes oldest-to-newest order."""

    queue_depth: float
    verify_delete_backlog: float
    outage_declared: bool = False


@dataclass(frozen=True)
class SaturationBounds:
    # Fewer than this many observations can't support a trend judgement at all.
    min_observations: int = 4
    # Minimum growth (mean of the window's second half minus its first half) in queue
    # depth to count as "trending upward". Small relative to saturation_probe.py's
    # unacked_window_depth_growth_threshold (20.0): this axis ramps single-digit
    # concurrent Bridge processes, not hundreds of connections, so its queue depth lives
    # on a correspondingly smaller scale.
    queue_depth_growth_threshold: float = 2.0
    # Same head/tail growth split, for the verified-but-undeleted backlog.
    verify_delete_backlog_growth_threshold: float = 2.0


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

    for signal_check in (_queue_depth_signal, _verify_delete_backlog_signal):
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


def _queue_depth_signal(window: Sequence[Observation], bounds: SaturationBounds) -> Verdict | None:
    head, tail = _split_halves(window)
    if not head or not tail:
        return None
    head_mean = fmean(o.queue_depth for o in head)
    tail_mean = fmean(o.queue_depth for o in tail)
    growth = tail_mean - head_mean
    if growth < bounds.queue_depth_growth_threshold:
        return None
    return Verdict(
        status=VerdictStatus.SATURATED,
        binding_signal=Signal.QUEUE_DEPTH,
        reason=(
            f"upload queue depth grew by {growth:.1f} file(s) from the window's first "
            f"half to its second, at or above the {bounds.queue_depth_growth_threshold} "
            f"bound"
        ),
        figures={
            "queue_depth_head_mean": head_mean,
            "queue_depth_tail_mean": tail_mean,
            "queue_depth_growth": growth,
        },
    )


def _verify_delete_backlog_signal(
    window: Sequence[Observation], bounds: SaturationBounds
) -> Verdict | None:
    # A growing verify-then-delete backlog during a declared outage (the object-storage
    # Destination is down) is expected, not a capacity signal — see the module
    # docstring. Suppressed for the whole window, same reasoning as
    # saturation_probe.py's disk-buffer check.
    if any(o.outage_declared for o in window):
        return None
    head, tail = _split_halves(window)
    if not head or not tail:
        return None
    head_mean = fmean(o.verify_delete_backlog for o in head)
    tail_mean = fmean(o.verify_delete_backlog for o in tail)
    growth = tail_mean - head_mean
    if growth < bounds.verify_delete_backlog_growth_threshold:
        return None
    return Verdict(
        status=VerdictStatus.SATURATED,
        binding_signal=Signal.VERIFY_DELETE_BACKLOG,
        reason=(
            f"verify-then-delete backlog grew by {growth:.1f} file(s) at steady state "
            f"(no outage declared in the window), at or above the "
            f"{bounds.verify_delete_backlog_growth_threshold} bound"
        ),
        figures={
            "verify_delete_backlog_head_mean": head_mean,
            "verify_delete_backlog_tail_mean": tail_mean,
            "verify_delete_backlog_growth": growth,
        },
    )
