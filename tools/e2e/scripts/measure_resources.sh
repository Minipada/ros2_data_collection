#!/bin/bash
# Samples CPU/RSS of the `dc` container at the reference workload, informational per
# the PRD (not a gate). Run by tools/e2e/scripts/run.sh in the background for the
# duration of a harness run; kill it (SIGTERM) to stop sampling.
#
# Usage: measure_resources.sh <output.csv> [container-name]
set -euo pipefail

OUT="${1:?usage: measure_resources.sh <output.csv> [container-name]}"
DC_C="${2:-dc_e2e_dc}"
echo "timestamp,cpu_percent,mem_usage,mem_percent" > "$OUT"

trap 'exit 0' TERM INT

while true; do
  # The container isn't running during the outage/restart window, so `podman stats` can
  # legitimately fail or return nothing — handle that via the if-condition (which errexit
  # ignores) rather than masking every failure with `|| true`.
  if LINE="$(podman stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}}' \
      --filter "name=$DC_C" 2>/dev/null)" && [ -n "$LINE" ]; then
    echo "$(date -u +%Y-%m-%dT%H:%M:%SZ),$LINE" >> "$OUT"
  fi
  sleep 5
done
