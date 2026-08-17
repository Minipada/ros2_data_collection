#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Simulation smoke check (#279). From the repo root:
#
#   ./tools/sim/scripts/run.sh
#
# Runs the warehouse simulation headless in a container from the DC workspace image and
# asserts it actually works: the robot spawns, every bridged topic carries usable data,
# /cmd_vel moves the robot, Nav2 comes up and localizes, and the waypoint follower makes
# real progress through the QR-coded aisles.
#
# Why this exists as its own thing: none of the simulation/demo layer is reachable by
# `colcon build` or `colcon test`. It is SDF, launch files, bridge configs and params --
# data, not code -- so a robot that never spawns, a lidar that aborts the server, a
# bridge pointed at a dead topic and a nav params file full of Humble-era keys all
# passed CI cleanly (#279, #324, #325). The E2E harness deliberately doesn't help here:
# it feeds the pipeline a synthetic workload precisely so its zero-loss proof doesn't
# depend on a simulator.
#
# Slow by nature: the warehouse is 317 model instances and CI has no GPU, so the world
# runs well under real time (see dc_simulation/README.md for measured factors), and
# `detect` slows it further by running the DC pipeline alongside. ci.yaml's `sim` job runs
# every stage on each PR with a handful of waypoints; only the full 60-waypoint pass is
# too slow for that, and that is what the daily .github/workflows/sim.yaml runs.
#
# Env vars (all optional):
#   DC_SIM_IMAGE            prebuilt workspace image to run (default dc-workspace:latest).
#                           Both CI jobs set this to the image they built or pulled.
#   DC_SIM_WAYPOINTS        waypoints the follower must reach before the run is
#                           considered good (default 6). A handful proves planning,
#                           control, localization, the goal checker and -- with `detect`
#                           -- QR-code reading all work; 60 is the full pass through every
#                           QR-coded pallet, which the daily job runs.
#   DC_SIM_WAYPOINT_TIMEOUT seconds to allow for that progress (default 2700).
#   DC_SIM_STAGES           space-separated subset of: lint sim nav waypoints detect
#                           (default: all five). `detect` is a modifier on
#                           `waypoints`: it brings the DC pipeline up with the
#                           demo and additionally asserts that a QR code was
#                           actually read at the stations the robot visited,
#                           which is the only check that fails when the cameras
#                           and the waypoints drift apart (#51). It costs one
#                           extra process, not an extra run.
#   DC_SIM_KEEP             "true" to leave the last stage's container up for inspection.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(dirname "$(dirname "$SIM_DIR")")"
RUN_DIR="$SIM_DIR/.run"

IMAGE="${DC_SIM_IMAGE:-dc-workspace:latest}"
WAYPOINTS="${DC_SIM_WAYPOINTS:-6}"
WAYPOINT_TIMEOUT="${DC_SIM_WAYPOINT_TIMEOUT:-2700}"
STAGES="${DC_SIM_STAGES:-lint sim nav waypoints detect}"
KEEP="${DC_SIM_KEEP:-false}"
CONTAINER=""

log() { printf '\n\033[1m[sim] %s\033[0m\n' "$*"; }
fail() { printf '\n\033[1;31m[sim] FAILED: %s\033[0m\n' "$*" >&2; exit 1; }
has_stage() { [[ " $STAGES " == *" $1 "* ]]; }

# These arrive from workflow_dispatch inputs, so a typo should say so rather than surface
# as an arithmetic error a thousand seconds into the run.
[[ "$WAYPOINTS" =~ ^[0-9]+$ ]] || fail "DC_SIM_WAYPOINTS must be a number, got '$WAYPOINTS'"
[[ "$WAYPOINT_TIMEOUT" =~ ^[0-9]+$ ]] || fail "DC_SIM_WAYPOINT_TIMEOUT must be a number, got '$WAYPOINT_TIMEOUT'"
for stage in $STAGES; do
  case "$stage" in
    lint | sim | nav | waypoints | detect) ;;
    *) fail "unknown stage '$stage' in DC_SIM_STAGES (want: lint sim nav waypoints detect)" ;;
  esac
done

cleanup() {
  local rc=$?
  # Nothing started yet (input validation rejected something, say) — and stop_stack is
  # defined below this trap, so it is not safe to call in that window either.
  [ -n "$CONTAINER" ] || exit $rc
  if [ "$KEEP" = "true" ]; then
    log "DC_SIM_KEEP=true — leaving $CONTAINER running"
    exit $rc
  fi
  stop_stack
  exit $rc
}
trap cleanup EXIT

# --- containers -----------------------------------------------------------------------
# One container per stage, torn down before the next starts. Tempting as it looks to
# reuse one and `pkill` the simulator between stages, that does not work: pkill -f
# matches the very shell running it, so the kill takes out the killer and leaves gz sim
# and its bridge orphaned. A second stack then joins the same ROS graph, two nodes
# publish /clock, and every tf2 buffer starts reporting "Detected jump back in time"
# while Nav2 quietly never localizes. A container boundary makes teardown total.
SOURCE='source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash'

start_stack() {
  CONTAINER="dc-sim-$$-$1"
  log "starting $CONTAINER from $IMAGE"
  podman run -d --name "$CONTAINER" --shm-size=2g \
    -v "$REPO_ROOT/tools/sim/scripts:/opt/sim:ro,Z" \
    "$IMAGE" sleep infinity >/dev/null
}

stop_stack() {
  [ -n "$CONTAINER" ] || return 0
  mkdir -p "$RUN_DIR"
  podman exec "$CONTAINER" bash -lc 'cat /tmp/*.log 2>/dev/null' >> "$RUN_DIR/sim.log" 2>/dev/null || true
  podman rm -f "$CONTAINER" >/dev/null 2>&1 || true
  CONTAINER=""
}

# `ros2 launch` needs a writable HOME for its log directory.
rin() { podman exec -e HOME=/root "$CONTAINER" bash -lc "$1"; }
rbg() { podman exec -d -e HOME=/root "$CONTAINER" bash -lc "$1" >/dev/null; }

# --- stage: static lint ---------------------------------------------------------------
if has_stage lint; then
  start_stack lint
  log "linting launch files, SDF and the bridge config (seconds, no simulator)"
  rin "$SOURCE && python3 /opt/sim/lint_launch_files.py" || fail "static lint"
  stop_stack
fi

# Waits for a condition inside the container, polling until a deadline.
wait_for() {
  local desc="$1" timeout="$2" cmd="$3" waited=0
  log "waiting for $desc (up to ${timeout}s)"
  while [ "$waited" -lt "$timeout" ]; do
    if rin "$cmd" >/dev/null 2>&1; then
      log "$desc — ready after ${waited}s"
      return 0
    fi
    sleep 15
    waited=$((waited + 15))
  done
  return 1
}

# --- stage: simulation only -----------------------------------------------------------
if has_stage sim; then
  start_stack sim
  log "launching warehouse.launch.py headless"
  rbg "$SOURCE && ros2 launch dc_simulation warehouse.launch.py headless:=True > /tmp/warehouse.log 2>&1"

  # The robot appearing in the world is the single most load-bearing assertion here:
  # an illegal-SDF spawn rejection is silent apart from one [Err] line (#324).
  wait_for "the robot to spawn" 600 \
    "source /opt/ros/jazzy/setup.bash && gz model --list 2>/dev/null | grep -q turtlebot3_waffle" \
    || fail "turtlebot3_waffle never appeared in gz model --list"

  rin "$SOURCE && python3 /opt/sim/verify_sim.py topics" || fail "topic checks"
  rin "$SOURCE && python3 /opt/sim/verify_sim.py cmd_vel" || fail "cmd_vel round trip"

  stop_stack
fi

# --- stage: simulation + Nav2 ---------------------------------------------------------
if has_stage nav || has_stage waypoints || has_stage detect; then
  start_stack nav
  # `detect` needs the measurement/bridge half of the demo running; the other stages do
  # not, and leaving it out keeps them cheaper on a runner with no GPU.
  if has_stage detect; then USE_DC=True; else USE_DC=False; fi
  log "launching tb3_qrcodes.launch.py (simulation + Nav2, DC $USE_DC)"
  rbg "$SOURCE && ros2 launch dc_demos tb3_qrcodes.launch.py headless:=True use_rviz:=False use_dc:=$USE_DC > /tmp/qrcodes.log 2>&1"

  wait_for "Nav2 to activate" 1800 \
    'grep -aq "lifecycle_manager_navigation.*Managed nodes are active" /tmp/qrcodes.log' \
    || fail "nav2 never reported all managed nodes active"
  rin 'grep -aq "lifecycle_manager_localization.*Managed nodes are active" /tmp/qrcodes.log' \
    || fail "localization never reported all managed nodes active"

  rin "$SOURCE && python3 /opt/sim/verify_sim.py nav" || fail "nav checks"
fi

# --- stage: waypoint following --------------------------------------------------------
if has_stage waypoints || has_stage detect; then
  if has_stage detect; then
    log "recording every QR code the demo reads"
    rbg "$SOURCE && python3 /opt/sim/verify_sim.py record > /tmp/codes.log 2>&1"
  fi
  log "running qrcodes_waypoint_follower until it reaches waypoint $WAYPOINTS/60"
  rbg "$SOURCE && ros2 run dc_demos qrcodes_waypoint_follower > /tmp/waypoints.log 2>&1; echo EXIT=\$? >> /tmp/waypoints.log"

  # Progress, not completion: the follower reaching waypoint N proves planning, control,
  # localization and the goal checker all work. Completion of all 60 is an hour-plus and
  # belongs in a manual run (DC_SIM_WAYPOINTS=60).
  # Reading a number out of the container is fiddlier than it looks under
  # `set -euo pipefail`: grep exits 1 when it matches nothing (so a bare command
  # substitution kills the script the first time round, before the follower has logged
  # anything), and `grep -c` prints "0" *and* exits 1 (so a `|| echo 0` fallback yields
  # "0\n0", which every later [ ] test then chokes on). Swallow the status inside, and
  # return digits or nothing at all.
  count_in_container() {
    local out
    out="$(rin "$1" 2>/dev/null || true)"
    printf '%s' "$out" | head -1 | tr -dc '0-9'
  }

  waited=0
  reached=0
  while [ "$waited" -lt "$WAYPOINT_TIMEOUT" ]; do
    reached=$(count_in_container 'grep -ao "Executing current waypoint: [0-9]*" /tmp/waypoints.log | tail -1 | grep -o "[0-9]*$"')
    reached="${reached:-0}"
    if [ "$reached" -ge "$WAYPOINTS" ]; then
      log "reached waypoint $reached/60 after ${waited}s"
      break
    fi
    if rin 'grep -aq "^EXIT=" /tmp/waypoints.log' 2>/dev/null; then
      if rin 'grep -aq "EXIT=0" /tmp/waypoints.log' 2>/dev/null; then
        log "follower completed the full pass"
        reached=60
        break
      fi
      fail "waypoint follower exited non-zero (see $RUN_DIR/sim.log)"
    fi
    sleep 30
    waited=$((waited + 30))
  done
  [ "$reached" -ge "$WAYPOINTS" ] || fail "only reached waypoint $reached/60 in ${WAYPOINT_TIMEOUT}s (wanted $WAYPOINTS)"

  # Navigation failures don't necessarily stop progress, so assert on them separately.
  aborted=$(count_in_container 'grep -acE "Goal was aborted|Goal failed|Failed to make progress" /tmp/qrcodes.log')
  [ "${aborted:-0}" -eq 0 ] || fail "$aborted navigation failures in the Nav2 log"
  log "no aborted or failed navigation goals"
fi

# --- stage: QR-code detection ---------------------------------------------------------
# Navigating the aisles perfectly while reading nothing is exactly the failure #51
# describes, and every other stage here passes while it happens: the robot spawns, the
# cameras publish, Nav2 reaches every goal. Only counting decoded codes catches it.
if has_stage detect; then
  log "checking the demo actually read the codes it drove up to"

  # One station is one stop in front of one pallet, and the Camera measurements report
  # once per stop (condition_max_measurements: 1 against the `moving` condition), so the
  # count tracks the stations visited. Asserting on `reached - 1` rather than `reached`
  # leaves room for the station the robot is standing at when the poll loop above breaks
  # out, and for the first goal, which it drives to from the spawn pose across the
  # warehouse rather than from a pallet.
  want=$((reached - 1))
  [ "$want" -ge 1 ] || want=1

  # Poll rather than read once. The loop above breaks the instant nav2 reports it is
  # working on the next waypoint, which is a second or two before the Camera measurement
  # at the station behind it has polled, decoded and published — reading the count there
  # and then would be a race, not an assertion.
  #
  # Count QRCode reads specifically. Two things make a looser count worthless: ZXing's
  # default options try every symbology, and a run of this check duly read a "51" off the
  # warehouse shelving before reaching the first pallet; and if the recorder died on
  # import, its traceback would read as a pile of successful detections.
  reads=0
  waited=0
  while [ "$waited" -lt 180 ]; do
    reads=$(count_in_container 'grep -ac "format=QRCode" /tmp/codes.log')
    reads="${reads:-0}"
    [ "$reads" -ge "$want" ] && break
    sleep 15
    waited=$((waited + 15))
  done

  codes=$(rin 'grep -a "format=QRCode" /tmp/codes.log | grep -ao "code=[0-9]*" | sort -u | tr "\n" " "' 2>/dev/null || true)
  log "$reads QR codes read over $reached stations: ${codes:-none}"
  [ "$reads" -ge "$want" ] || fail "only $reads QR codes read, wanted at least $want (see $RUN_DIR/sim.log)"

  # Pose estimation (#48). The recorder reports each map-frame code pose next to the
  # nearest QR model in the world file, so this asserts against the simulator's own
  # ground truth. The tolerance is deliberately loose: dc_simulation's qrcode_* asset
  # stretches a 290x365 texture over a square face, so its printed code is 0.362 m
  # across and 0.292 m down (lint_launch_files' measured CODE_HALF_W/CODE_HALF_H) and
  # no single code_size is exact. A metre still catches
  # every failure worth catching here -- wrong frame, flipped axis, uninitialised
  # intrinsics -- which a decoded-payload check cannot.
  #
  # Three distinguishable outcomes, because the first version of this check reported all
  # of them as "estimate_pose off, or no CameraInfo" and sent the investigation to the
  # wrong component: the real cause was the ground-truth reader returning nothing.
  poses=$(count_in_container 'grep -ac "code_pose=" /tmp/codes.log')
  best=$(rin 'grep -ao "gt_err=[0-9.]*" /tmp/codes.log | cut -d= -f2 | sort -g | head -1' 2>/dev/null | head -1 | tr -dc '0-9.' || true)
  if [ -n "${best:-}" ]; then
    log "best QR-code pose landed ${best} m from the world's own placement"
    awk -v e="$best" 'BEGIN { exit !(e < 1.0) }' \
      || fail "closest estimated code pose was ${best} m from any QR model in the world"
  elif [ "${poses:-0}" -gt 0 ]; then
    frames=$(rin 'grep -ao "frame=[^ ]*" /tmp/codes.log | sort -u | tr "\n" " "' 2>/dev/null || true)
    fail "$poses code poses recorded but none in the map frame (frames seen: ${frames:-none}); the pose_frame TF lookup or the world ground truth is failing"
  else
    fail "no estimated code poses recorded: estimate_pose off, no CameraInfo on the camera_info topic, or the solve rejected every detection"
  fi
fi

log "simulation check passed (stages: $STAGES)"
