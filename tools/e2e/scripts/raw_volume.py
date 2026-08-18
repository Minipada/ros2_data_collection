# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Stored-volume accounting for raw-mode (#227) Records.

Both users of it — verify_zero_loss.py's `check_raw()` (one Tag, the E2E harness) and
tools/sim/scripts/measure_raw_volume.sh's reporter (every Tag, the simulated robot) —
have to answer the same question the same way, or the doc's numbers and the harness's
numbers stop being comparable. What a Record *costs* is the bytes the Destination
actually wrote: the JSON line plus the newline the sink terminates it with.
"""

import datetime
import json


class Volume:
    """Bytes, Records and arrival span over one raw Destination's NDJSON output."""

    def __init__(self) -> None:
        self.records = 0
        self.stored_bytes = 0
        self.first_seen: datetime.datetime | None = None
        self.last_seen: datetime.datetime | None = None

    def add_line(self, line: str) -> None:
        """Count one stored line, newline included — what the sink wrote to disk."""
        self.records += 1
        self.stored_bytes += len(line) + 1

    def add_time(self, stored_at) -> None:
        """Extend the arrival span with a Record's sink timestamp, if it parses."""
        if not isinstance(stored_at, str):
            return
        try:
            when = datetime.datetime.fromisoformat(stored_at.replace("Z", "+00:00"))
        except ValueError:
            return
        if self.first_seen is None or when < self.first_seen:
            self.first_seen = when
        if self.last_seen is None or when > self.last_seen:
            self.last_seen = when

    @property
    def span_seconds(self) -> float | None:
        """Wall-clock seconds the Records arrived over, or None if unmeasurable.

        Under a second of arrivals is not a rate — dividing by it turns sampling noise
        into a throughput claim.
        """
        if self.first_seen is None or self.last_seen is None:
            return None
        span = (self.last_seen - self.first_seen).total_seconds()
        return span if span >= 1.0 else None

    def summary(self) -> dict:
        """Volume as the report keys both callers publish."""
        out = {
            "stored_bytes": self.stored_bytes,
            "bytes_per_record": self.stored_bytes // self.records if self.records else None,
        }
        span = self.span_seconds
        if span:
            out["span_seconds"] = round(span, 1)
            out["records_per_second"] = round(self.records / span, 2)
            out["bytes_per_second"] = round(self.stored_bytes / span, 1)
            out["projected_mb_per_day"] = round(self.stored_bytes / span * 86400 / 1e6, 1)
        return out


def project(summary: dict, sim_seconds: float, max_rate_hz: float) -> dict:
    """What one Tag's measured volume is worth on a robot running at real time (#326).

    A simulator that runs at a fraction of real time makes wall-clock throughput
    meaningless — every rate it produces is scaled by the real-time factor. Two things
    survive that scaling: the bytes a Record costs, and the rate the robot publishes at,
    which is measured in *simulated* seconds. Multiply them, after applying the rate
    limiter that would bind on a real robot, and the projection is the robot's, not the
    simulator's.

    Args:
        summary: one Tag's entry from `summarize_by_tag()`.
        sim_seconds: simulated seconds the measurement window covered.
        max_rate_hz: the profile's `raw.max_rate_hz` (0 = no limiter).
    """
    per_record = summary.get("bytes_per_record")
    if not per_record or sim_seconds <= 0:
        return {}
    published_hz = summary["records"] / sim_seconds
    shipped_hz = min(published_hz, max_rate_hz) if max_rate_hz else published_hz
    bytes_per_second = per_record * shipped_hz
    return {
        "published_hz": round(published_hz, 2),
        "shipped_hz": round(shipped_hz, 2),
        "projected_bytes_per_second": round(bytes_per_second, 1),
        "projected_gb_per_day": round(bytes_per_second * 86400 / 1e9, 3),
    }


def summarize_by_tag(path: str, arrival_rates: bool = True) -> dict:
    """Per-Tag and total volume over a raw Destination's NDJSON file.

    A line that isn't JSON still cost its bytes, so it counts toward the total under the
    `(unparsable)` Tag rather than vanishing from the accounting.

    Args:
        path: the Destination's newline-delimited JSON.
        arrival_rates: keep the timestamp-derived rates. Off for a simulated robot: the
            sink timestamp is the Record's `date`, which for a message carrying a
            `std_msgs/msg/Header` is the *message's* stamp — simulated time under a
            simulator, and mixed with the Bridge's wall clock for messages without a
            header. Spans across the two are meaningless; rates there come from the
            simulated window instead (`project()`).
    """
    total = Volume()
    per_tag: dict[str, Volume] = {}

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
                tag = event.get("tag") or "(untagged)"
                stored_at = event.get("timestamp")
            except json.JSONDecodeError:
                tag, stored_at = "(unparsable)", None
            total.add_line(line)
            if arrival_rates:
                total.add_time(stored_at)
            volume = per_tag.setdefault(tag, Volume())
            volume.add_line(line)
            if arrival_rates:
                volume.add_time(stored_at)

    return {
        "tags": {
            tag: {"records": v.records, **v.summary()}
            for tag, v in sorted(per_tag.items(), key=lambda kv: -kv[1].stored_bytes)
        },
        "total": {"records": total.records, **total.summary()},
    }
