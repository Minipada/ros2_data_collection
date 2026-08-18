#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# What raw-mode collection costs on a real robot (#326). From the repo root:
#
#   ./tools/sim/scripts/measure_raw_volume.sh
#
# Boots dc_simulation's warehouse world headless in a container from the DC workspace
# image, drives the TurtleBot3-Waffle in a slow circle, runs `dc_bridge` in raw mode
# (#227) against its live topics, and reports per-Tag and total volume — Records, bytes
# per Record, publish rate and a projected GB/day — for each of three configurations
# (see tools/sim/params/raw_*.yaml). Informational and local: a number nobody asserts a
# threshold on has no business gating a PR, and booting gz-sim would make every E2E run
# minutes slower. It is deliberately not wired into ci.yaml.
#
# It needs the same workspace image tools/sim/scripts/run.sh uses (build it with
# tools/e2e/scripts/build.sh, or set DC_SIM_IMAGE to one you have) and podman. An
# equivalent local ROS 2 Jazzy + gz-sim Harmonic workspace works too, if you would rather
# run the four steps by hand.
#
# Rates are measured in *simulated* seconds, not wall-clock ones. The warehouse is 317
# model instances with two RGBD cameras and a lidar, and a machine with no GPU runs it far
# under real time — a wall-clock throughput measured here would be the renderer's number,
# not the robot's. Bytes per Record are exact regardless, so the profiles collect with the
# rate limiter switched off (see the params files), the window is counted in simulated
# seconds, and the report multiplies bytes per Record by the rate raw mode would actually
# ship at on a robot running at real time — the publish rate, capped at the shipped
# `max_rate_hz` default. A run therefore takes as long in wall-clock as this machine needs.
#
# Env vars (all optional):
#   DC_SIM_IMAGE          workspace image to run (default dc-workspace:latest).
#   DC_RAW_PROFILES       space-separated subset of: defaults sensors camera (all three
#                         by default), each a file in tools/sim/params/raw_<name>.yaml.
#   DC_RAW_SIM_SECONDS    simulated seconds to measure over (default 10).
#   DC_RAW_CAMERA_SIM_SECONDS  the same for the `camera` profile (default 15). Ten
#                         1280x720 frames a simulated second is ~110 MB of JSON, which
#                         nothing downstream carries — expect the Bridge to shed most of
#                         it and the window to hold a handful of frames.
#   DC_RAW_WALL_TIMEOUT   wall-clock seconds a window may take before it is cut short and
#                         reported over the simulated time it did cover (default 1800).
#   DC_RAW_PROJECT_RATE_HZ  the `raw.max_rate_hz` the projection assumes (default 10, the
#                         value dc_bringup/params/dc_raw_params.yaml ships; 0 = none).
#   DC_SIM_KEEP           "true" to leave the last profile's container up for inspection.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$SIM_DIR/.run"

IMAGE="${DC_SIM_IMAGE:-dc-workspace:latest}"
PROFILES="${DC_RAW_PROFILES:-defaults sensors camera}"
SIM_SECONDS="${DC_RAW_SIM_SECONDS:-10}"
CAMERA_SIM_SECONDS="${DC_RAW_CAMERA_SIM_SECONDS:-15}"
WALL_TIMEOUT="${DC_RAW_WALL_TIMEOUT:-1800}"
PROJECT_RATE_HZ="${DC_RAW_PROJECT_RATE_HZ:-10}"
KEEP="${DC_SIM_KEEP:-false}"
CONTAINER=""

log() { printf '\n\033[1m[raw-volume] %s\033[0m\n' "$*"; }
fail() { printf '\n\033[1;31m[raw-volume] FAILED: %s\033[0m\n' "$*" >&2; exit 1; }

for profile in $PROFILES; do
  [ -f "$SIM_DIR/params/raw_$profile.yaml" ] || fail "no profile 'raw_$profile.yaml' in $SIM_DIR/params"
done
[[ "$SIM_SECONDS" =~ ^[0-9]+$ ]] || fail "DC_RAW_SIM_SECONDS must be a number, got '$SIM_SECONDS'"

cleanup() {
  local rc=$?
  [ -n "$CONTAINER" ] || exit $rc
  if [ "$KEEP" = "true" ]; then
    log "DC_SIM_KEEP=true — leaving $CONTAINER running"
    exit $rc
  fi
  podman rm -f "$CONTAINER" >/dev/null 2>&1 || true
  exit $rc
}
trap cleanup EXIT

SOURCE='source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash'
rin() { podman exec -e HOME=/root "$CONTAINER" bash -lc "$1"; }
rbg() { podman exec -d -e HOME=/root "$CONTAINER" bash -lc "$1" >/dev/null; }

# Simulated seconds, as a float, from the /clock the ros_gz_bridge republishes.
sim_now() {
  local out
  out="$(rin "$SOURCE && timeout 30 ros2 topic echo --once --field clock /clock 2>/dev/null" |
    awk '/^ *sec:/ { s = $2 } /^ *nanosec:/ { printf "%.3f", s + $2 / 1e9; exit }')"
  [ -n "$out" ] || fail "could not read /clock — the simulation is not publishing time"
  printf '%s' "$out"
}

mkdir -p "$RUN_DIR"

for profile in $PROFILES; do
  CONTAINER="dc-rawvol-$$-$profile"
  log "profile '$profile': starting $CONTAINER from $IMAGE"
  podman run -d --name "$CONTAINER" --shm-size=2g \
    -v "$SIM_DIR/params:/opt/params:ro,Z" \
    "$IMAGE" sleep infinity >/dev/null

  log "launching the warehouse world headless"
  rbg "$SOURCE && ros2 launch dc_simulation warehouse.launch.py headless:=True > /tmp/warehouse.log 2>&1"

  # Same load-bearing assertion as tools/sim/scripts/run.sh: an illegal-SDF spawn
  # rejection is one [Err] line, and everything downstream then measures an empty world.
  waited=0
  until rin "source /opt/ros/jazzy/setup.bash && timeout 30 gz model --list 2>/dev/null | grep -q turtlebot3_waffle" >/dev/null 2>&1; do
    [ "$waited" -lt 900 ] || fail "turtlebot3_waffle never appeared in gz model --list"
    sleep 15
    waited=$((waited + 15))
  done
  log "robot spawned after ${waited}s"

  # A stationary robot is the wrong thing to measure: odometry and TF of a robot that is
  # not moving are zeros, and a JSON zero is a fraction of the ~17 characters a real
  # double costs. Drive it in a slow circle instead. The publisher runs on wall time, so
  # /cmd_vel itself is excluded from collection (see the params files).
  rbg "$SOURCE && ros2 topic pub -r 1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}, angular: {z: 0.3}}' > /tmp/drive.log 2>&1"

  # Under the camera profile the graph query itself gets slow; a miss here only
  # costs the report its type column.
  rin "$SOURCE && timeout 180 ros2 topic list -t > /tmp/topics.txt 2>/dev/null" || true

  log "launching dc_bridge in raw mode (profile '$profile')"
  rbg "$SOURCE && ros2 launch dc_bringup dc_raw.launch.py dc_params_file:=/opt/params/raw_$profile.yaml > /tmp/bridge.log 2>&1"

  RAW=/root/.dc/raw/records.ndjson
  waited=0
  until rin "test -s $RAW" >/dev/null 2>&1; do
    [ "$waited" -lt 300 ] || fail "no raw Record reached the Destination in ${waited}s (see $RUN_DIR/$profile.bridge.log)"
    sleep 10
    waited=$((waited + 10))
  done
  log "first Record stored after ${waited}s — measuring"

  # The window is the lines that appeared between the two marks, so nothing has to be
  # killed to end it: the sink keeps appending, and the report reads only the slice. The
  # sim clock is sampled next to each line count so the two windows line up.
  if [ "$profile" = "camera" ]; then want="$CAMERA_SIM_SECONDS"; else want="$SIM_SECONDS"; fi
  line0="$(rin "wc -l < $RAW" | tr -dc '0-9')"
  sim0="$(sim_now)"
  wall0="$(date +%s)"
  while :; do
    sim1="$(sim_now)"
    wall1="$(date +%s)"
    elapsed_sim="$(awk -v a="$sim0" -v b="$sim1" 'BEGIN { printf "%.2f", b - a }')"
    if awk -v e="$elapsed_sim" -v w="$want" 'BEGIN { exit !(e >= w) }'; then break; fi
    if [ $((wall1 - wall0)) -ge "$WALL_TIMEOUT" ]; then
      log "wall-clock timeout after $((wall1 - wall0))s — reporting over ${elapsed_sim} simulated seconds"
      break
    fi
    sleep 20
  done
  line1="$(rin "wc -l < $RAW" | tr -dc '0-9')"
  log "${elapsed_sim}s of simulated time over $((wall1 - wall0))s wall, $((line1 - line0)) Records"

  rin "sed -n '$((line0 + 1)),${line1}p' $RAW" > "$RUN_DIR/$profile.ndjson"
  rin "cat /tmp/topics.txt" > "$RUN_DIR/$profile.topics.txt" 2>/dev/null || true
  rin "cat /tmp/bridge.log" > "$RUN_DIR/$profile.bridge.log" 2>/dev/null || true
  # The Bridge's own drop counters. Collection here is uncapped on purpose, so anything
  # but zeros means the measurement saw fewer messages than the robot published.
  rin "$SOURCE && timeout 60 ros2 service call /dc_bridge/ready std_srvs/srv/Trigger" \
    > "$RUN_DIR/$profile.ready.txt" 2>&1 || true
  counters="$(grep -o "raw: .*" "$RUN_DIR/$profile.ready.txt" | tail -1 || true)"
  log "bridge counters — ${counters:-unavailable}"
  if [ -z "$counters" ]; then
    log "WARNING: could not read the Bridge's counters — drops would be invisible"
  else
    shed="$(printf '%s' "$counters" |
      awk '{ f = 0; d = 0
             for (i = 1; i <= NF; i++) {
               if ($i == "forwarded,") f = $(i - 1)
               if ($i ~ /^(rate|oversize|shipper|undecodable)/) d += $(i - 1)
             }
             if (f + d > 0) printf "%d %.2f", d, 100 * d / (f + d) }')"
    read -r dropped share <<< "$shed"
    if [ "${dropped:-0}" -gt 0 ]; then
      log "WARNING: the Bridge shed $dropped Record(s) ($share% of what it saw) — the figures below undercount by that much"
    fi
  fi

  python3 "$SCRIPT_DIR/report_raw_volume.py" "$RUN_DIR/$profile.ndjson" \
    --profile "$profile" \
    --sim-seconds "$elapsed_sim" \
    --wall-seconds "$((wall1 - wall0))" \
    --max-rate-hz "$PROJECT_RATE_HZ" \
    --topic-types "$RUN_DIR/$profile.topics.txt" \
    --json "$RUN_DIR/$profile.volume.json" || fail "reporting profile '$profile'"

  if [ "$KEEP" != "true" ]; then
    podman rm -f "$CONTAINER" >/dev/null 2>&1 || true
    CONTAINER=""
  fi
done

log "done — reports in $RUN_DIR/*.volume.json"
