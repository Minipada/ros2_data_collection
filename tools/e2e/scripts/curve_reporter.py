# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Curve reporter (#380): turns a set of ramp runs into a stable, diffable structured
report plus a human-readable summary, for #323's limits harness.

A "run" is one axis's ramp (shipper fan-in, single-robot ceiling, uploader concurrency,
drain rate, ...): the ascending sequence of levels tried, whether each was actually
driven and measured or is a modelled point beyond what was run, the real-vs-synthetic
composition behind each level (per the PRD: "every reported figure states its
real-versus-synthetic composition"), and the CPU/memory/disk usage sampled at each
measured level. `build_report()` is a pure function of that data — no I/O, no
containers, no dependency on `ramp_controller`/`saturation_probe` — matching this
module's siblings in the epic.

Two things the PRD calls out as load-bearing:

- **Stability.** Identical input must produce identical output, and the report's axis
  order must not depend on the order runs were handed in — a diff between two runs
  should only ever show a real change. `build_report()` sorts axes by name for this.
- **Honesty.** A measured point and an extrapolated (projected) one must never look
  alike. Every `LevelResult` carries its own `PointKind`, both the structured report and
  `render_summary()` print it next to the level, and an extrapolated point never carries
  a resource sample — there is nothing measured to report for a level that wasn't run.
"""

from __future__ import annotations

import dataclasses
import json
from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import StrEnum


class PointKind(StrEnum):
    MEASURED = "measured"
    EXTRAPOLATED = "extrapolated"


class ResourceKind(StrEnum):
    CPU = "cpu"
    MEMORY = "memory"
    DISK = "disk"


class RunOutcome(StrEnum):
    """Mirrors ramp_controller.RampOutcome's values without importing that module —
    same decoupling the ramp controller uses for saturation_probe.Verdict."""

    KNEE_FOUND = "knee_found"
    BOUND_NOT_FOUND = "bound_not_found"


@dataclass(frozen=True)
class ResourceSample:
    """CPU/memory/disk usage accompanying one measured level, from the existing
    resource sampler. Percentages, not clamped to 100 — `podman stats`' CPU figure can
    exceed 100% on a multi-core container."""

    cpu_percent: float
    mem_percent: float
    disk_percent: float

    def __post_init__(self) -> None:
        for name, value in (
            ("cpu_percent", self.cpu_percent),
            ("mem_percent", self.mem_percent),
            ("disk_percent", self.disk_percent),
        ):
            if value < 0:
                raise ValueError(f"{name} cannot be negative, got {value}")


@dataclass(frozen=True)
class RunComposition:
    """How many real DC stacks vs. synthetic senders made up one reported level."""

    real_stacks: int
    synthetic_senders: int

    def __post_init__(self) -> None:
        if self.real_stacks < 0 or self.synthetic_senders < 0:
            raise ValueError("composition counts cannot be negative")

    @property
    def total(self) -> int:
        return self.real_stacks + self.synthetic_senders


@dataclass(frozen=True)
class LevelResult:
    """One level tried (or projected) on an axis's ramp.

    `resources` is `None` for an extrapolated point by construction of the reporter's
    contract: there is no sample to report for a level that was never driven. A measured
    point normally carries one, but isn't required to (a run whose resource sampler
    wasn't wired up still reports a level; it just can't back a binding-constraint
    claim from it).
    """

    level: float
    saturated: bool
    kind: PointKind
    composition: RunComposition
    resources: ResourceSample | None = None


@dataclass(frozen=True)
class AxisRun:
    """One axis's full ramp, as input to `build_report()`."""

    axis: str
    unit: str
    outcome: RunOutcome
    # The highest level tried whose verdict stayed clear; None only when the very
    # first level tried already saturated. Mirrors ramp_controller.RampResult.
    highest_clear_level: float | None
    # None exactly when outcome is BOUND_NOT_FOUND — never a fabricated ceiling.
    tripped_level: float | None
    points: Sequence[LevelResult]

    def __post_init__(self) -> None:
        if not self.axis:
            raise ValueError("an axis run must name its axis")
        if len(self.points) == 0:
            raise ValueError(f"axis run {self.axis!r} must carry at least one point")


@dataclass(frozen=True)
class BindingConstraint:
    resource: ResourceKind | None
    percent: float | None
    reason: str


@dataclass(frozen=True)
class AxisReport:
    axis: str
    unit: str
    outcome: RunOutcome
    highest_clear_level: float | None
    tripped_level: float | None
    binding_constraint: BindingConstraint
    points: tuple[LevelResult, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class CurveReport:
    axes: tuple[AxisReport, ...] = field(default_factory=tuple)


# CPU checked before memory before disk: both the priority order the PRD lists the
# three signals in, and the deterministic tie-break when two resources peak at the
# same percentage.
_RESOURCE_PRIORITY = (ResourceKind.CPU, ResourceKind.MEMORY, ResourceKind.DISK)


def _resource_percent(sample: ResourceSample, kind: ResourceKind) -> float:
    return {
        ResourceKind.CPU: sample.cpu_percent,
        ResourceKind.MEMORY: sample.mem_percent,
        ResourceKind.DISK: sample.disk_percent,
    }[kind]


def _binding_constraint(run: AxisRun) -> BindingConstraint:
    """The resource whose usage peaked highest across this run's measured points —
    "correctly identified from the resource-usage series accompanying a run" per the
    issue. Never guesses one for a run that didn't saturate, and never reports a
    constraint sourced from an extrapolated point."""
    if run.outcome != RunOutcome.KNEE_FOUND:
        return BindingConstraint(
            resource=None,
            percent=None,
            reason="no saturation observed within tested range; no binding constraint to report",
        )

    measured = [p for p in run.points if p.kind == PointKind.MEASURED and p.resources is not None]
    if not measured:
        return BindingConstraint(
            resource=None,
            percent=None,
            reason="knee found but no measured resource sample accompanies this run",
        )

    best_kind: ResourceKind | None = None
    best_percent = -1.0
    best_level: float | None = None
    for kind in _RESOURCE_PRIORITY:
        for point in measured:
            assert point.resources is not None  # narrowed by the filter above
            percent = _resource_percent(point.resources, kind)
            if percent > best_percent:
                best_kind, best_percent, best_level = kind, percent, point.level

    assert best_kind is not None
    return BindingConstraint(
        resource=best_kind,
        percent=best_percent,
        reason=f"{best_kind.value} peaked at {best_percent:.1f}% at level {best_level} (measured)",
    )


def build_report(runs: Sequence[AxisRun]) -> CurveReport:
    """The curve reporter's entry point: a pure function from a set of per-axis runs
    to a stable report. Axes are sorted by name and each axis's points by level, so the
    same set of runs always yields the same report regardless of the order they were
    collected or handed in — required for the report to be diffable between two runs."""
    axes = tuple(_to_axis_report(run) for run in sorted(runs, key=lambda r: r.axis))
    return CurveReport(axes=axes)


def _to_axis_report(run: AxisRun) -> AxisReport:
    points = tuple(sorted(run.points, key=lambda p: p.level))
    return AxisReport(
        axis=run.axis,
        unit=run.unit,
        outcome=run.outcome,
        highest_clear_level=run.highest_clear_level,
        tripped_level=run.tripped_level,
        binding_constraint=_binding_constraint(run),
        points=points,
    )


def to_json(report: CurveReport) -> str:
    """The structured report, as stable/diffable JSON: sorted keys so key order can
    never be the source of a spurious diff, on top of `build_report()`'s own sorting of
    axes and points."""
    return json.dumps(dataclasses.asdict(report), indent=2, sort_keys=True)


def _resource_sample_from_dict(d: dict | None) -> ResourceSample | None:
    return None if d is None else ResourceSample(**d)


def _binding_constraint_from_dict(d: dict) -> BindingConstraint:
    return BindingConstraint(
        resource=None if d["resource"] is None else ResourceKind(d["resource"]),
        percent=d["percent"],
        reason=d["reason"],
    )


def _level_result_from_dict(d: dict) -> LevelResult:
    return LevelResult(
        level=d["level"],
        saturated=d["saturated"],
        kind=PointKind(d["kind"]),
        composition=RunComposition(**d["composition"]),
        resources=_resource_sample_from_dict(d["resources"]),
    )


def axis_report_from_dict(d: dict) -> AxisReport:
    """The inverse of `dataclasses.asdict()` for one axis, used to read back a
    per-axis report a scenario script already wrote via `to_json()` — the shape #386's
    one-command entry point combines across axes."""
    return AxisReport(
        axis=d["axis"],
        unit=d["unit"],
        outcome=RunOutcome(d["outcome"]),
        highest_clear_level=d["highest_clear_level"],
        tripped_level=d["tripped_level"],
        binding_constraint=_binding_constraint_from_dict(d["binding_constraint"]),
        points=tuple(_level_result_from_dict(p) for p in d["points"]),
    )


def from_json(text: str) -> CurveReport:
    """The inverse of `to_json()`."""
    data = json.loads(text)
    return CurveReport(axes=tuple(axis_report_from_dict(a) for a in data["axes"]))


def render_summary(report: CurveReport) -> str:
    """A human-readable summary naming, per axis, the outcome and the binding
    constraint at saturation, with every point labelled measured or extrapolated and
    its real/synthetic composition — never presenting a projection as a measurement."""
    lines = ["Limits harness curve report", "=" * len("Limits harness curve report"), ""]
    for axis in report.axes:
        lines.append(f"{axis.axis} ({axis.unit}):")
        clear = "none" if axis.highest_clear_level is None else axis.highest_clear_level
        if axis.outcome == RunOutcome.KNEE_FOUND:
            lines.append(f"  knee found at {axis.tripped_level} (highest clear: {clear})")
        else:
            lines.append(f"  bound not found within tested range (highest clear: {clear})")

        bc = axis.binding_constraint
        if bc.resource is None:
            lines.append(f"  binding constraint: none — {bc.reason}")
        else:
            lines.append(
                f"  binding constraint: {bc.resource.value} at {bc.percent:.1f}% ({bc.reason})"
            )

        lines.append("  points:")
        for point in axis.points:
            status = "saturated" if point.saturated else "clear"
            comp = point.composition
            composition_text = f"{comp.real_stacks} real + {comp.synthetic_senders} synthetic"
            if point.resources is None:
                resource_text = "not measured"
            else:
                r = point.resources
                resource_text = (
                    f"cpu={r.cpu_percent:.1f}% mem={r.mem_percent:.1f}% disk={r.disk_percent:.1f}%"
                )
            lines.append(
                f"    - {point.level} {axis.unit}: {status} [{point.kind.value}] "
                f"({composition_text}) {resource_text}"
            )
        lines.append("")

    return "\n".join(lines).rstrip() + "\n"


def _point_at(axis: AxisReport, level: float) -> LevelResult:
    for point in axis.points:
        if point.level == level:
            return point
    raise ValueError(f"axis {axis.axis!r} has no point at level {level}")


def closing_sentence(axis: AxisReport) -> str:
    """#323's PRD's single defensible sentence, generalised across axes: 'sustains N
    <unit> before applying backpressure, with its composition and measured/extrapolated
    conditions stated' — never a fabricated ceiling. `run_limits_shipper_fanin.py`
    carries its own bespoke wording for the PRD's literal flagship example; this is the
    generic form #386's one-command entry point uses for every axis, built entirely from
    the already-computed `AxisReport`, with no axis-specific knowledge."""
    unit = axis.unit
    if axis.outcome == RunOutcome.BOUND_NOT_FOUND:
        highest = axis.highest_clear_level
        if highest is None:
            return (
                f"{axis.axis}: bound not found — no level was tested without saturating, "
                f"so no ceiling is reported (never fabricated)."
            )
        point = _point_at(axis, highest)
        comp = point.composition
        return (
            f"{axis.axis} sustains at least {highest} {unit} ({comp.real_stacks} real + "
            f"{comp.synthetic_senders} synthetic, {point.kind.value}) without applying "
            f"backpressure — the ramp never saturated within the tested range, so no "
            f"ceiling is reported beyond {highest} {unit} (bound not found; never "
            f"fabricated)."
        )

    tripped = axis.tripped_level
    assert tripped is not None  # KNEE_FOUND always carries a tripped_level
    tripped_point = _point_at(axis, tripped)
    extrapolated_note = (
        "not extrapolated — the knee itself was measured, not projected"
        if tripped_point.kind == PointKind.MEASURED
        else "extrapolated beyond what was directly measured"
    )
    highest = axis.highest_clear_level
    if highest is None:
        return (
            f"{axis.axis} saturated at the first level tested, {tripped} {unit} "
            f"({tripped_point.composition.real_stacks} real + "
            f"{tripped_point.composition.synthetic_senders} synthetic); no lower level "
            f"was sustained ({extrapolated_note})."
        )
    sustain_point = _point_at(axis, highest)
    comp = sustain_point.composition
    return (
        f"{axis.axis} sustains {highest} {unit} ({comp.real_stacks} real + "
        f"{comp.synthetic_senders} synthetic, {sustain_point.kind.value}) before applying "
        f"backpressure; saturation observed at {tripped} {unit} ({extrapolated_note})."
    )
