#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Load driver byte-compatibility integration test (#378). From the repo root:
#
#   ./tools/e2e/scripts/run_load_driver_shipper_test.sh
#
# tools/e2e/scripts/load_driver.py's own unit tests (tools/e2e/test/test_load_driver.py)
# prove its wire format and rate/ack/ledger behaviour against an in-process fake fluent
# listener, standalone — but "the Shipper's ingest listener cannot distinguish it from a
# real Bridge at the socket" (#378's own framing) is a claim only a real Shipper build
# can settle, never a mock. This script is that check: it runs the pinned Vector version
# `vector_vendor/CMakeLists.txt` builds dc_bridge against (as a container — the public
# `timberio/vector` image, not a repo-built artifact, since vector_vendor only vendors a
# bare binary at colcon build time, not a runnable container image) with a bare `fluent`
# source (global acknowledgements enabled, exactly as dc_bridge/src/render.cpp turns it
# on for the real Bridge) plus a `file` sink recording every Record it actually decoded,
# points load_driver.py at it, and asserts two things a mock cannot prove:
#
#   1. every frame the driver sent was accepted and acknowledged (the driver's own
#      summary reports zero unacked at the end of the run);
#   2. the driver's ledger and Vector's own decoded output name the exact same set of
#      (connection_id, seq) pairs -- not a count that happens to match, the same records,
#      neither missing an entry from one side nor carrying an extra one.
#
# Not wired into ci.yaml, same as run_retention.sh/run_degraded.sh -- a sibling
# standalone scenario, not a mode inside the zero-loss run.
#
# Env vars: DC_E2E_LOAD_DRIVER_CONNECTIONS (default 20), DC_E2E_LOAD_DRIVER_RATE_HZ
# (per-connection Records/s, default 20), DC_E2E_LOAD_DRIVER_DURATION_S (default 3),
# DC_E2E_LOAD_DRIVER_PORT (default 24224, the fluent protocol's conventional port),
# DC_E2E_KEEP ("true" to leave the Vector container and .run artifacts in place after a
# failure for debugging, matching every other scenario script's own knob).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(cd "$E2E_DIR/../.." && pwd)"
RUN_DIR="$E2E_DIR/.run/load_driver_shipper_test"

CONNECTIONS="${DC_E2E_LOAD_DRIVER_CONNECTIONS:-20}"
RATE_HZ="${DC_E2E_LOAD_DRIVER_RATE_HZ:-20}"
DURATION_S="${DC_E2E_LOAD_DRIVER_DURATION_S:-3}"
PORT="${DC_E2E_LOAD_DRIVER_PORT:-24224}"
TAG="dc.e2e.load_driver_shipper_test"
KEEP="${DC_E2E_KEEP:-false}"

VECTOR_C=dc_e2e_ldtest_vector

# The exact version dc_bridge is built and tested against -- read out of the CMake file
# that pins it, not duplicated here as a second source of truth that could drift from it.
VECTOR_VERSION="$(grep -oP 'set\(VECTOR_VERSION "\K[^"]+' "$REPO_ROOT/vector_vendor/CMakeLists.txt")"
VECTOR_IMAGE="docker.io/timberio/vector:${VECTOR_VERSION}-debian"

log() { echo "[load-driver-shipper-test $(date -u +%H:%M:%S)] $*"; }

cleanup() {
  local exit_code=$?
  if [ "$exit_code" -ne 0 ] && [ "$KEEP" = "true" ]; then
    log "FAILED (exit $exit_code) -- leaving the Vector container and $RUN_DIR up (DC_E2E_KEEP=true) for debugging"
    exit "$exit_code"
  fi
  log "tearing down"
  if podman container exists "$VECTOR_C"; then
    podman logs "$VECTOR_C" > "$RUN_DIR/vector.log" 2>&1 || true
  fi
  podman rm -f --ignore "$VECTOR_C" >/dev/null
  exit "$exit_code"
}
trap cleanup EXIT

rm -rf "$RUN_DIR"
mkdir -p "$RUN_DIR/output"

cat > "$RUN_DIR/vector.toml" <<EOF
data_dir = "/var/lib/vector"

[acknowledgements]
enabled = true

[sources.fluent_in]
type = "fluent"
address = "0.0.0.0:${PORT}"

[sinks.out]
type = "file"
inputs = ["fluent_in"]
path = "/output/records.ndjson"
encoding.codec = "json"
EOF

log "pulling the pinned Vector build ($VECTOR_IMAGE)"
podman pull -q "$VECTOR_IMAGE" >/dev/null

log "starting Vector with a bare fluent source (port $PORT) + a file sink"
podman run -d --network host --name "$VECTOR_C" \
  -v "$RUN_DIR/vector.toml:/etc/vector/vector.toml:Z" \
  -v "$RUN_DIR/output:/output:Z" \
  "$VECTOR_IMAGE" -c /etc/vector/vector.toml >/dev/null

log "waiting for the fluent source to accept connections"
if ! timeout 30 bash -c "until (exec 3<>/dev/tcp/127.0.0.1/${PORT}) 2>/dev/null; do sleep 0.5; done"; then
  log "FAIL: Vector's fluent source never started listening on port $PORT"
  exit 1
fi

log "running load_driver.py: $CONNECTIONS connections @ ${RATE_HZ}Hz for ${DURATION_S}s"
SUMMARY_JSON="$(cd "$REPO_ROOT" && uv run --frozen python3 tools/e2e/scripts/load_driver.py \
  127.0.0.1 "$PORT" \
  --connections "$CONNECTIONS" \
  --rate "$RATE_HZ" \
  --duration "$DURATION_S" \
  --tag "$TAG" \
  --ledger-out "$RUN_DIR/ledger.jsonl")"
echo "$SUMMARY_JSON" > "$RUN_DIR/driver_summary.json"
log "driver summary: $SUMMARY_JSON"

# Give the file sink's own batching a moment to flush the last events to disk.
sleep 2
podman stop "$VECTOR_C" >/dev/null

SENT="$(python3 -c "import json,sys; print(json.load(sys.stdin)['sent'])" <<<"$SUMMARY_JSON")"
UNACKED="$(python3 -c "import json,sys; print(json.load(sys.stdin)['unacked'])" <<<"$SUMMARY_JSON")"

if [ "$SENT" -eq 0 ]; then
  log "FAIL: the driver reports it sent 0 frames -- nothing was exercised"
  exit 1
fi
if [ "$UNACKED" -ne 0 ]; then
  log "FAIL: $UNACKED of $SENT frame(s) were never acknowledged by the real Shipper build"
  exit 1
fi
log "OK: all $SENT frames were accepted and acknowledged"

log "comparing the driver's ledger against what Vector actually decoded"
python3 - "$RUN_DIR/ledger.jsonl" "$RUN_DIR/output/records.ndjson" "$TAG" <<'PYEOF'
import json
import sys

ledger_path, received_path, expected_tag = sys.argv[1:4]

with open(ledger_path) as f:
    sent = {(entry["connection_id"], entry["seq"]) for entry in map(json.loads, f)}

received = set()
wrong_tag = []
with open(received_path) as f:
    for line in f:
        record = json.loads(line)
        if record.get("tag") != expected_tag:
            wrong_tag.append(record)
            continue
        received.add((record["connection_id"], record["seq"]))

missing = sent - received  # the driver sent these; Vector never decoded them
extra = received - sent  # Vector decoded these; the driver's ledger has no record of them

if wrong_tag:
    print(f"FAIL: {len(wrong_tag)} received record(s) carry a tag other than {expected_tag!r}")
    sys.exit(1)
if missing:
    print(f"FAIL: {len(missing)} ledger entr(ies) never showed up in Vector's output: "
          f"{sorted(missing)[:10]}")
    sys.exit(1)
if extra:
    print(f"FAIL: {len(extra)} record(s) Vector decoded have no matching ledger entry: "
          f"{sorted(extra)[:10]}")
    sys.exit(1)
if not sent:
    print("FAIL: empty ledger -- nothing to compare")
    sys.exit(1)

print(f"OK: all {len(sent)} ledger entries match Vector's decoded output one-to-one")
PYEOF

log "PASS"
