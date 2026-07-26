#!/bin/bash
# Entrypoint for the tools/e2e/Containerfile "dc" image: starts the synthetic
# reference-workload generator (tools/e2e/scripts/workload_generator.py) and the real
# DC stack (dc_bringup.launch.py) side by side, exactly like a robot would run
# Measurements alongside real sensor drivers.
set -euo pipefail

# shellcheck disable=SC1091
source /root/ws/install/setup.bash

# Backgrounded (not `exec`'d) so the TERM/INT trap below — set on this script's own
# shell — actually fires: `exec` replaces the shell process image, and a replaced
# process doesn't run its predecessor's traps, which would leak both children when
# podman/systemd stops the container.
python3 /opt/e2e/workload_generator.py &
GEN_PID=$!

ros2 launch dc_bringup dc_bringup.launch.py \
  dc_params_file:=/opt/e2e/e2e_params.yaml \
  "$@" &
LAUNCH_PID=$!

cleanup() {
  kill "$LAUNCH_PID" "$GEN_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" "$GEN_PID" 2>/dev/null || true
}
trap cleanup TERM INT

set +e
wait "$LAUNCH_PID"
EXIT_CODE=$?
set -e
cleanup
exit "$EXIT_CODE"
