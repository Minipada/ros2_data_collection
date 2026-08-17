#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Entrypoint for the tools/e2e/Containerfile "dc" image: starts the synthetic
# reference-workload generator (tools/e2e/scripts/workload_generator.py) and the real
# DC stack (dc_bringup.launch.py) side by side, exactly like a robot would run
# Measurements alongside real sensor drivers.
set -euo pipefail

# colcon/ROS setup files reference unset vars (COLCON_TRACE, AMENT_TRACE_SETUP_FILES,
# …); sourcing them under `set -u` aborts the entrypoint ("COLCON_TRACE: unbound
# variable") before the stack ever launches. Source with nounset off, then restore it.
set +u
# shellcheck disable=SC1091
source /root/ws/install/setup.bash
set -u

# Backgrounded (not `exec`'d) so the TERM/INT trap below — set on this script's own
# shell — actually fires: `exec` replaces the shell process image, and a replaced
# process doesn't run its predecessor's traps, which would leak both children when
# podman/systemd stops the container.
#
# Started first, before dc_bringup: params/e2e_mcap_sink.toml's Vector `socket` sink
# connects to it (ADR-0009, #210), and while Vector retries a not-yet-listening sink
# without dropping data (buffered per e2e_mcap_sink.toml's own disk buffer), starting
# the listener first means the very first synth02/synth03 Record doesn't have to rely
# on that retry at all.
#
# `python3 -m dc_mcap_writer.cli`, deliberately not `ros2 run dc_mcap_writer
# dc_mcap_writer`: `ros2 run` spawns the target as a *child* of its own Python process
# rather than exec-replacing itself, and does not forward signals to that child — `kill
# "$MCAP_PID"` below would only ever reach the `ros2 run` wrapper, leaving the actual
# writer process running, orphaned, until the container runtime kills it outright
# (never running its own graceful-shutdown path, so its currently-open .mcap is left
# without the footer that makes it readable). Confirmed empirically: `kill -TERM` on
# `ros2 run`'s own PID does not stop the dc_mcap_writer process it started; `kill -TERM`
# on `python3 -m dc_mcap_writer.cli`'s PID does, immediately. dc_mcap_writer has no
# rclpy/ROS-node dependency of its own, so nothing here actually needs `ros2 run`'s
# node-launching machinery — `install/setup.bash` already puts it on PYTHONPATH.
#
# --max-duration-secs 15, well under this default of 300s: a `.mcap` file only becomes
# readable once its rotation finishes, so frequent rotation bounds how much a genuinely
# abrupt kill (this harness's outage/restart still exercises SIGKILL races even with
# the `ros2 run` bug fixed — that's the container runtime's own grace-period behavior,
# not something dc_mcap_writer can fully rule out) can cost, leaving every
# already-finished segment safe regardless.
python3 -m dc_mcap_writer.cli \
  --output-dir /root/.dc/e2e/data/mcap \
  --host 127.0.0.1 --port 9191 \
  --max-duration-secs 15 &
MCAP_PID=$!

python3 /opt/e2e/workload_generator.py &
GEN_PID=$!

ros2 launch dc_bringup dc_bringup.launch.py \
  dc_params_file:=/opt/e2e/e2e_params.yaml \
  "$@" &
LAUNCH_PID=$!

cleanup() {
  # Best-effort shutdown: errexit is scoped off here (not masked per-command with
  # `|| true`) because signalling or reaping a child that has already exited returns
  # non-zero, which is expected during teardown — not an error to surface. dc_mcap_writer
  # only finishes a readable .mcap file (writes its Data End record/index) on a clean
  # shutdown (SIGTERM/SIGINT) — see dc_mcap_writer/dc_mcap_writer/cli.py — so it must be
  # signalled and waited on here like the other two, not left for the container runtime
  # to SIGKILL once this script exits.
  set +e
  kill "$LAUNCH_PID" "$GEN_PID" "$MCAP_PID" 2>/dev/null
  wait "$LAUNCH_PID" "$GEN_PID" "$MCAP_PID" 2>/dev/null
}
trap cleanup TERM INT

set +e
wait "$LAUNCH_PID"
EXIT_CODE=$?
set -e
cleanup
exit "$EXIT_CODE"
