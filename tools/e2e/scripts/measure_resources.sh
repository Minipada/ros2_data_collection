#!/bin/bash
# Samples CPU/RSS of the `dc` container at the reference workload, informational per
# the PRD (not a gate). Run by tools/e2e/scripts/run.sh in the background for the
# duration of a harness run; kill it (SIGTERM) to stop sampling.
#
# Usage: measure_resources.sh <output.csv>
set -euo pipefail

OUT="${1:?usage: measure_resources.sh <output.csv>}"
echo "timestamp,cpu_percent,mem_usage,mem_percent" > "$OUT"

trap 'exit 0' TERM INT

while true; do
  LINE="$(podman stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}}' \
    --filter name=dc-dc-1 2>/dev/null || true)"
  if [ -z "$LINE" ]; then
    # podman compose's generated container name can vary by compose provider/version
    # (dc-dc-1, e2e_dc_1, ...) — fall back to matching any container from this
    # project's "dc" service by image name instead of guessing the exact name.
    LINE="$(podman stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}}' \
      --filter ancestor=localhost/e2e-dc 2>/dev/null | head -1 || true)"
  fi
  if [ -n "$LINE" ]; then
    echo "$(date -u +%Y-%m-%dT%H:%M:%SZ),$LINE" >> "$OUT"
  fi
  sleep 5
done
