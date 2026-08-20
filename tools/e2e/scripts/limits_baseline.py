# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Limits harness: one-command capacity baseline (#386, part of #323's epic).

Combines the four axes' already-written `curve_reporter.CurveReport` JSON files
(Shipper fan-in, single-robot ceiling, Uploader concurrency, drain rate — each written
by its own axis script via `curve_reporter.to_json()`) into one combined, diffable
report plus a human-readable summary carrying every axis's closing sentence
(`curve_reporter.closing_sentence()`). `merge_reports()`/`render_baseline()` are pure
functions of the per-axis reports already on disk — no containers, no ramping, no
axis-specific knowledge — so they're unit-tested directly against synthetic
`CurveReport`s, matching this epic's other reporting code
(`curve_reporter.build_report()`'s own testing precedent). Reading the four report files
and writing the combined ones is the only I/O here; `run_limits_capacity_baseline.sh`
is what actually runs the four axes first.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from curve_reporter import CurveReport, closing_sentence, from_json, render_summary, to_json


def merge_reports(reports: Sequence[CurveReport]) -> CurveReport:
    """One combined report from N single-axis (or multi-axis) reports. Axis order is
    resolved by name alone, independent of the order the reports were handed in or of
    each axis's own internal ordering — the same stability guarantee
    `curve_reporter.build_report()` gives within one report, extended across reports."""
    axes = tuple(sorted((axis for report in reports for axis in report.axes), key=lambda a: a.axis))
    names = [axis.axis for axis in axes]
    if len(names) != len(set(names)):
        duplicates = sorted({name for name in names if names.count(name) > 1})
        raise ValueError(f"duplicate axis name(s) across the reports being combined: {duplicates}")
    return CurveReport(axes=axes)


def render_baseline(report: CurveReport) -> str:
    """The combined artifact's human-readable form: `curve_reporter.render_summary()`'s
    per-axis detail, followed by every axis's one-sentence capacity claim in one place —
    #386's acceptance criterion that each axis's closing sentence, with its composition
    and measured/extrapolated conditions, is present in the combined output."""
    lines = [render_summary(report).rstrip(), "", "Closing statements:", ""]
    for axis in report.axes:
        lines.append(f"- {closing_sentence(axis)}")
    return "\n".join(lines).rstrip() + "\n"


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--report",
        dest="reports",
        action="append",
        required=True,
        metavar="PATH",
        help="a per-axis curve_reporter report.json; repeat once per axis",
    )
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-txt", required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])

    reports = [from_json(Path(p).read_text()) for p in args.reports]
    combined = merge_reports(reports)

    Path(args.output_json).write_text(to_json(combined))
    summary = render_baseline(combined)
    Path(args.output_txt).write_text(summary)
    print(summary)

    print(json.dumps({"axes": [axis.axis for axis in combined.axes]}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
