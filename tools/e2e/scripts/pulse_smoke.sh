#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Container-side half of the TopicStatistics smoke (tools/e2e/scripts/run_pulse.sh runs
# this with --entrypoint, like entrypoint.sh serves run.sh): pulse_bridge, an
# LD_PRELOAD'd producer, and the DC stack with pulse_smoke_params.yaml side by side, then
# the assertion subscriber. This script owns the processes and their teardown;
# run_pulse.sh owns the image ref and the captured logs.
set -euo pipefail

RECORD_TIMEOUT_SECONDS="${DC_E2E_RECORD_TIMEOUT_SECONDS:-120}"
PRODUCER_RATE_HZ="${DC_E2E_PRODUCER_RATE_HZ:-10}"
PUBLISH_PERIOD_S="${DC_E2E_PUBLISH_PERIOD_S:-5.0}"
STATS_LOG=/tmp/pulse.log

# colcon/ROS setup files reference unset vars (see entrypoint.sh) — source with nounset
# off, then restore it.
set +u
# shellcheck disable=SC1091
source /root/ws/install/setup.bash
# shellcheck disable=SC1091
source /opt/ros2_pulse/install/setup.bash
set -u

# Fail before anything is launched on the one thing the smoke cannot recover from: a
# tracetools build without instrumentation, which the probe counts through — no
# ros_trace symbols means no window is ever written (ros2_pulse README's own check).
TRACE_COUNT="$(nm -D "$(ros2 pkg prefix tracetools)"/lib/libtracetools.so* 2>/dev/null | grep -c ros_trace || true)"
if [ "${TRACE_COUNT}" -eq 0 ]; then
  echo "FAIL: libtracetools.so exposes no ros_trace symbols — ros2_pulse has nothing to hook into"
  exit 1
fi
echo "PASS: tracetools instrumentation present (${TRACE_COUNT} ros_trace symbols)"

cleanup() {
  # Best-effort teardown (see entrypoint.sh): a child that already exited returns
  # non-zero when signalled or waited on, which is teardown, not an error to surface.
  set +e
  kill "$PULSE_BRIDGE_PID" "$PRODUCER_PID" "$LAUNCH_PID" 2>/dev/null
  wait "$PULSE_BRIDGE_PID" "$PRODUCER_PID" "$LAUNCH_PID" 2>/dev/null
}
# Empty until launched, so a TERM racing the first launch cannot trip nounset in here.
PULSE_BRIDGE_PID=""
PRODUCER_PID=""
LAUNCH_PID=""
trap cleanup TERM INT EXIT

# Explicit `files` rather than pulse_bridge's $TMPDIR glob: the pairing with the
# producer's ROS_TOPIC_STATS_OUTPUT_FILE below is then visible here, not implicit.
ros2 run ros2_pulse pulse_bridge --ros-args -p "files:=['${STATS_LOG}']" &
PULSE_BRIDGE_PID=$!

# Text format is the probe's default and pulse_bridge refuses text logs — jsonl is what
# makes the bridge publish at all (its source refuses per file with one warning).
LD_PRELOAD="$(ros2 pkg prefix ros2_pulse)/lib/libros2_pulse.so" \
  ROS_TOPIC_STATS_OUTPUT_FILE="${STATS_LOG}" \
  ROS_TOPIC_STATS_FORMAT=jsonl \
  ROS_TOPIC_STATISTICS_PUBLISH_PERIOD="${PUBLISH_PERIOD_S}" \
  ros2 topic pub --rate "${PRODUCER_RATE_HZ}" /pulse/chatter std_msgs/msg/String '{data: pulse}' &
PRODUCER_PID=$!

ros2 launch dc_bringup dc_bringup.launch.py dc_params_file:=/opt/e2e/pulse_smoke_params.yaml &
LAUNCH_PID=$!

echo "waiting up to ${RECORD_TIMEOUT_SECONDS}s for a TopicStatistics Record (window=${PUBLISH_PERIOD_S}s + poll + stack startup)"
if python3 /opt/e2e/expect_topic_statistics_record.py --timeout-seconds "${RECORD_TIMEOUT_SECONDS}"; then
  echo "PASS: TopicStatistics E2E smoke (live ros2_pulse producer -> Record)"
else
  echo "FAIL: pulse log tail (empty means the producer never wrote a window; see the bridge and DC logs above)"
  tail -5 "${STATS_LOG}" 2>/dev/null || true
  exit 1
fi
