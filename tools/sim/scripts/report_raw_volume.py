#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Per-Tag volume report for the sim-backed raw-mode benchmark (#326).

Reads the NDJSON one profile's `file` Destination collected (see
tools/sim/scripts/measure_raw_volume.sh) and prints what raw mode costs, per Tag and in
total. The accounting itself is tools/e2e/scripts/raw_volume.py — the same code the E2E
harness's zero-loss verifier reports `stored_bytes`/`bytes_per_record` with, so the two
sets of numbers mean the same thing.

Rates are computed in *simulated* seconds and the projection is the robot's, not this
machine's: see `raw_volume.project()`.
"""

import argparse
import json
import os
import sys

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        "e2e",
        "scripts",
    ),
)

import raw_volume


def topic_of(tag: str, prefix: str = "dc.raw.") -> str:
    """The topic a raw Tag came from — the Tag mapping of raw_topics.md, reversed."""
    return "/" + tag[len(prefix) :].replace(".", "/") if tag.startswith(prefix) else tag


def human(n: float) -> str:
    for unit, scale in (("GB", 1e9), ("MB", 1e6), ("kB", 1e3)):
        if n >= scale:
            return f"{n / scale:.2f} {unit}"
    return f"{n:.0f} B"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ndjson", help="the profile's raw Destination output")
    parser.add_argument("--profile", default="", help="profile name, for the heading")
    parser.add_argument(
        "--sim-seconds", type=float, required=True, help="simulated seconds the window covered"
    )
    parser.add_argument(
        "--wall-seconds", type=float, required=True, help="wall-clock seconds it took"
    )
    parser.add_argument(
        "--max-rate-hz",
        type=float,
        default=10.0,
        help="the profile's raw.max_rate_hz, applied to the projection (0 = unlimited)",
    )
    parser.add_argument(
        "--topic-types", help="`ros2 topic list -t` output, to name each Tag's message type"
    )
    parser.add_argument("--json", help="also write the report as JSON here")
    args = parser.parse_args()

    if not os.path.exists(args.ndjson) or os.path.getsize(args.ndjson) == 0:
        print(f"no raw Records at {args.ndjson} — the profile collected nothing", file=sys.stderr)
        return 1

    types = {}
    if args.topic_types and os.path.exists(args.topic_types):
        for line in open(args.topic_types):
            topic, _, rest = line.strip().partition(" [")
            if rest.endswith("]"):
                types[topic] = rest[:-1]

    report = raw_volume.summarize_by_tag(args.ndjson, arrival_rates=False)
    rtf = args.sim_seconds / args.wall_seconds if args.wall_seconds else 0.0
    report["window"] = {
        "profile": args.profile,
        "sim_seconds": round(args.sim_seconds, 2),
        "wall_seconds": round(args.wall_seconds, 1),
        "real_time_factor": round(rtf, 4),
        "max_rate_hz": args.max_rate_hz,
    }

    total_projected = 0.0
    for tag, summary in report["tags"].items():
        summary["topic"] = topic_of(tag)
        summary["message_type"] = types.get(summary["topic"], "?")
        summary.update(raw_volume.project(summary, args.sim_seconds, args.max_rate_hz))
        total_projected += summary.get("projected_bytes_per_second", 0.0)
    report["total"]["projected_bytes_per_second"] = round(total_projected, 1)
    report["total"]["projected_gb_per_day"] = round(total_projected * 86400 / 1e9, 3)

    heading = f"raw-mode volume — profile '{args.profile}'" if args.profile else "raw-mode volume"
    print(f"\n{heading}")
    print(
        f"  window: {args.sim_seconds:.1f} simulated seconds over {args.wall_seconds:.0f} s "
        f"wall (real-time factor {rtf:.3f}), max_rate_hz {args.max_rate_hz:g}"
    )
    header = (
        f"{'topic':<46} {'type':<34} {'recs':>6} {'B/rec':>9} {'Hz':>7} {'B/s':>11} {'GB/day':>8}"
    )
    print(f"\n{header}\n{'-' * len(header)}")
    for summary in report["tags"].values():
        print(
            f"{summary['topic']:<46} {summary['message_type']:<34} {summary['records']:>6} "
            f"{summary['bytes_per_record'] or 0:>9} {summary.get('published_hz', 0):>7.1f} "
            f"{summary.get('projected_bytes_per_second', 0):>11,.0f} "
            f"{summary.get('projected_gb_per_day', 0):>8.3f}"
        )
    print(f"{'-' * len(header)}")
    total = report["total"]
    print(
        f"{'TOTAL':<46} {'':<34} {total['records']:>6} {'':>9} {'':>7} "
        f"{total['projected_bytes_per_second']:>11,.0f} {total['projected_gb_per_day']:>8.3f}"
    )
    print(
        f"\n  stored: {human(total['stored_bytes'])} in this window; projected "
        f"{human(total['projected_bytes_per_second'] * 3600)}/hour, "
        f"{total['projected_gb_per_day']:.2f} GB/day"
    )

    # Which topics the projection's rate limiter actually decimates — the difference
    # between what the robot publishes and what raw mode would ship.
    capped = [
        f"{s['topic']} ({s['published_hz']:g} Hz)"
        for s in report["tags"].values()
        if args.max_rate_hz and s.get("published_hz", 0) > args.max_rate_hz
    ]
    if capped:
        print(
            f"\n  decimated to {args.max_rate_hz:g} Hz by max_rate_hz in the projection: "
            f"{', '.join(capped)}"
        )

    if args.json:
        with open(args.json, "w") as f:
            json.dump(report, f, indent=2)
        print(f"\n  report: {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
