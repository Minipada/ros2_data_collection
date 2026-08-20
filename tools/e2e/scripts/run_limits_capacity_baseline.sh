#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Limits harness: one-command capacity baseline (#386, the final piece of #323's epic).
# From the repo root:
#
#   ./tools/e2e/scripts/run_limits_capacity_baseline.sh
#
# Runs all four limits-harness axes — Shipper fan-in (#382), single-robot ceiling
# (#383), Uploader concurrency (#384), drain rate (#385) — one after another (each is
# its own full podman bring-up/teardown, and no two of them are meant to run
# concurrently, same as every other pair of sibling scenario scripts in this harness),
# then combines the four already-written curve_reporter.py reports into one artifact via
# scripts/limits_baseline.py: a single combined_report.json (stable/diffable — a sort by
# axis name on top of each axis's own stable report, nothing more) and a
# combined_summary.txt carrying every axis's closing sentence
# (curve_reporter.closing_sentence()) alongside its per-axis detail. That's the artifact
# a contributor diffs to check the effect of a change, and the one a release records as
# its capacity baseline instead of repeating the previous release's claim.
#
# Only the Shipper fan-in axis's UNCONSTRAINED phase is folded into the combined
# report — its own deliberately-constrained second phase is that axis's internal
# instrument proof (#382's own gate, run and asserted here as part of running that
# axis), not a fifth axis of this baseline.
#
# Each axis script already fails loudly and aborts on its own gate (a hard-fail process
# exit, not a skip) — combined with this script's own `set -euo pipefail`, any axis
# failing aborts the whole baseline immediately: a combined artifact is only ever
# written from four axes that actually passed.
#
# Every env var each individual axis script accepts (DC_E2E_FANIN_*, DC_E2E_CEILING_*,
# DC_E2E_UPLOAD_*, DC_E2E_DRAIN_*, DC_E2E_IMAGE / DC_WORKSPACE_IMAGE, DC_E2E_KEEP) still
# applies here unchanged — this script does no parameter translation of its own, it only
# sequences the four scripts and combines what they already produce. See each axis
# script's own header for its env vars.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run/limits_capacity_baseline"

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

log() { echo "[e2e-baseline $(date -u +%H:%M:%S)] $*"; }

log "=== axis 1/4: Shipper fan-in (#382) ==="
"$SCRIPT_DIR/run_limits_shipper_fanin.sh"

log "=== axis 2/4: single-robot ceiling (#383) ==="
"$SCRIPT_DIR/run_limits_single_robot_ceiling.sh"

log "=== axis 3/4: Uploader concurrency (#384) ==="
"$SCRIPT_DIR/run_limits_upload_concurrency.sh"

log "=== axis 4/4: drain rate (#385) ==="
"$SCRIPT_DIR/run_limits_drain_rate.sh"

log "combining the four axes' reports into one capacity baseline"
uv run --frozen python3 "$SCRIPT_DIR/limits_baseline.py" \
  --report "$E2E_DIR/.run/limits_shipper_fanin/unconstrained_report.json" \
  --report "$E2E_DIR/.run/limits_single_robot_ceiling/curve_report.json" \
  --report "$E2E_DIR/.run/upload_concurrency/curve_report.json" \
  --report "$E2E_DIR/.run/limits_drain_rate/curve_report.json" \
  --output-json "$RUN_DIR/combined_report.json" \
  --output-txt "$RUN_DIR/combined_summary.txt"

echo
echo "=== #323's capacity baseline (#386) ==="
cat "$RUN_DIR/combined_summary.txt"

log "PASS: limits harness one-command capacity baseline (#386) — $RUN_DIR/combined_report.json"
