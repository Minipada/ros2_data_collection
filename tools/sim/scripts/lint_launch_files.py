#!/usr/bin/env python3
"""Static checks over every launch file and SDF in the simulation/demo layer.

Neither `colcon build` nor `colcon test` looks at any of this: launch files, SDF and
bridge configs are data, so a launch argument defaulting to a file Jazzy deleted, or a
model with two identically-named collisions, compiles and tests perfectly while the demo
is dead on arrival (#279, #324).

Runs inside the container against the *installed* share directories, which is what a
user's `ros2 launch` actually reads.
"""
import os
import re
import subprocess
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

ANSI = re.compile(r"\x1b\[[0-9;]*m")

PACKAGES = ["dc_demos", "dc_simulation", "dc_bringup"]
# Paths a launch argument may legitimately point at without the file existing yet.
ALLOW_MISSING = ()


class Report:
    def __init__(self):
        self.failures = []

    def check(self, ok, name, detail=""):
        print(f"  {'PASS' if ok else 'FAIL'}  {name}{': ' + detail if detail else ''}")
        if not ok:
            self.failures.append(name)

    def finish(self):
        if self.failures:
            print(f"\nFAILED ({len(self.failures)}): {', '.join(self.failures)}")
            return 1
        print("\nall checks passed")
        return 0


def launch_files():
    for pkg in PACKAGES:
        share = Path(get_package_share_directory(pkg))
        for launch in sorted((share / "launch").glob("*.launch.py")):
            yield pkg, launch


def check_launch_loads(report):
    """Every launch file loads, and every path-shaped default it advertises exists.

    `os.path.join` happily builds a path to a file that was deleted upstream, so the
    only way to catch it is to resolve the advertised defaults. This is what would have
    flagged nav2_bringup/worlds/world_only.model and maps/turtlebot3_world.yaml, both
    gone from Jazzy's nav2_bringup.

    Args:
        report: collects pass/fail for the exit status.
    """
    for pkg, path in launch_files():
        proc = subprocess.run(
            ["ros2", "launch", pkg, path.name, "--show-args"],
            capture_output=True,
            text=True,
            timeout=180,
        )
        report.check(proc.returncode == 0, f"{pkg}/{path.name} loads", proc.stderr.strip()[-200:])
        if proc.returncode != 0:
            continue

        missing = []
        for line in proc.stdout.splitlines():
            line = line.strip()
            if not line.startswith("(default: '") or not line.endswith("')"):
                continue
            value = line[len("(default: '") : -len("')")]
            if not value.startswith("/") or value in ALLOW_MISSING:
                continue
            # Only judge values that name a file; directories and expandable
            # strftime/$HOME patterns are not ours to resolve here.
            if any(c in value for c in "%$*") or "." not in Path(value).name:
                continue
            if not os.path.exists(value):
                missing.append(value)
        report.check(not missing, f"{pkg}/{path.name} default paths exist", "; ".join(missing))


def check_sdf(report):
    """gz's own parser validates every world and model dc_simulation ships.

    Catches the illegal SDF that Gazebo Classic tolerated and gz-sim does not -- two
    <collision> blocks with the same name inside one <link> made gz-sim reject the whole
    robot spawn, so the warehouse ran for two weeks with no robot in it (#324).

    Args:
        report: collects pass/fail for the exit status.
    """
    share = Path(get_package_share_directory("dc_simulation"))
    targets = sorted((share / "worlds").glob("*.world")) + sorted(
        (share / "worlds").glob("*.model")
    )
    report.check(bool(targets), "found worlds/models to validate", f"{len(targets)} files")

    # `gz sdf` resolves model:// through SDF_PATH, not through the GZ_SIM_RESOURCE_PATH
    # dc_simulation's env hook sets — without it every <include> in the world reports
    # "Unable to find uri". Feeding it the same path makes the check meaningful in both
    # directions: it validates the structure *and* that every included model resolves.
    env = dict(os.environ)
    env["SDF_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", str(share / "models"))

    for target in targets:
        proc = subprocess.run(
            ["gz", "sdf", "-k", str(target)],
            capture_output=True,
            text=True,
            timeout=300,
            env=env,
        )
        errors = sdf_errors(proc.stderr + proc.stdout)
        report.check(not errors, f"{target.name} is valid SDF", "; ".join(errors[:3]))


def sdf_errors(output):
    """The structural errors in `gz sdf -k` output, and only those.

    Three things make this less obvious than it looks, all verified against a model with
    a deliberately reintroduced duplicate collision:

    - it exits 0 even when it reports an error, so the return code says nothing;
    - libsdformat's own structural errors are the `Error Code N: Msg: ...` lines, which
      is what has to be matched;
    - a world full of `<include><uri>model://…` produces a flood of `Tried to use
      callback in sdf::findFile()` — the standalone parser has no resource resolver, so
      those are noise about this tool, not about the file. `<gz_frame_id>` likewise
      warns as "not defined in SDF" because it is a gz-sim runtime extension.

    Args:
        output: combined stdout+stderr of `gz sdf -k`.

    Returns:
        The `Error Code N: ...` lines, empty when the file is valid.
    """
    clean = ANSI.sub("", output)
    return [
        line.strip()
        for line in clean.replace("Error Code", "\nError Code").splitlines()
        if line.strip().startswith("Error Code")
    ]


def check_bridge_config(report):
    """Every ROS topic dc_measurements' demo params subscribe to is bridged.

    The qrcodes demo shipped for two weeks naming camera topics the bridge published
    under different names, so the Camera measurements had nothing to subscribe to.

    Args:
        report: collects pass/fail for the exit status.
    """
    import yaml

    sim = Path(get_package_share_directory("dc_simulation"))
    bridged = set()
    for entry in yaml.safe_load((sim / "config" / "warehouse_bridge.yaml").read_text()):
        name = entry["ros_topic_name"]
        bridged.add(name if name.startswith("/") else "/" + name)

    demos = Path(get_package_share_directory("dc_demos"))
    for params in sorted((demos / "params").glob("qrcodes_*.yaml")):
        raw = yaml.safe_load(params.read_text())
        server = raw.get("measurement_server", {}).get("ros__parameters", {})
        wanted = {
            v["cam_topic"] for v in server.values() if isinstance(v, dict) and "cam_topic" in v
        }
        missing = sorted(wanted - bridged)
        report.check(
            not missing,
            f"{params.name} cam_topics are bridged",
            "; ".join(missing),
        )


# --- QR-code alignment ------------------------------------------------------------------
#
# The demo's waypoints are camera stations: the robot stops there so that a camera can
# read the QR code on the pallet in front of it. Nothing in the repo used to relate the
# two, so the stations drifted out of the cameras' field of view and stayed there for
# years (#51) -- a failure invisible to every other check, because the robot navigates
# perfectly and the cameras publish perfectly good pictures of the wrong thing.
#
# The numbers below are measured, not assumed: rendering a code from a known pose and
# reading back the bounding box ZXing reports for it gives
CODE_CENTRE_Z = 0.539  # height of the symbol's centre above the floor, m
CODE_HALF_W = 0.181  # half the symbol's rendered width, m
CODE_HALF_H = 0.146  # half its rendered height, m
# The map frame is the world frame shifted by this much. Measured off qrcodes.pgm: the
# four pallet blocks sit at map x -18.100/-16.800/-14.300/-13.000 against world
# -10.8/-9.5/-7.0/-5.7, and the first pallet row at map y -10.475 against world -11.9.
WORLD_TO_MAP = (-7.300, 1.425)
# sdformat's default when a <camera> declares no <horizontal_fov>, which neither of
# waffle.model's does.
DEFAULT_HFOV = 1.047


def read_camera_poses():
    """Where waffle.model's two inspection cameras sit, relative to base_footprint.

    An SDF <sensor> pose is relative to its parent <link>, so both have to be composed --
    the bug this check exists for was exactly that composition being overlooked, leaving
    both cameras 133 mm ahead of the point the waypoints aim at.

    Returns:
        {"left": (x, y, z, yaw), "right": ...}, metres and radians.
    """
    import xml.etree.ElementTree as ElementTree

    share = Path(get_package_share_directory("dc_simulation"))
    model = ElementTree.parse(share / "worlds" / "waffle.model").getroot().find("model")
    poses = {}
    for side in ("left", "right"):
        link = model.find(f'link[@name="{side}_camera_link"]')
        # The link's own <pose>, not <inertial>'s or <collision>'s: find() only looks at
        # direct children, which is the whole point of using it here.
        base = [float(v) for v in link.find("pose").text.split()]
        rel = [float(v) for v in link.find("sensor").find("pose").text.split()]
        # Both poses are yaw-only in this model, so composing them is an addition.
        poses[side] = (
            base[0] + rel[0],
            base[1] + rel[1],
            base[2] + rel[2],
            base[5] + rel[5],
        )
    return poses


def read_code_planes():
    """The QR-coded pallet faces in qrcodes.world, as (map x, facing, {y}).

    Returns:
        A list of (plane_x, normal_x, ys): the face's x in the map frame, the direction
        its codes look along x (+1 or -1), and the set of y values it carries.
    """
    share = Path(get_package_share_directory("dc_simulation"))
    world = re.sub(r"<!--.*?-->", "", (share / "worlds" / "qrcodes.world").read_text(), flags=re.S)
    planes = {}
    for _, body in re.findall(r'<model name="([^"]+)">(.*?)</model>', world, re.S):
        uri = re.search(r"<uri>model://(qrcode_[^<]+)</uri>", body)
        pose = re.search(r"<pose>([^<]+)</pose>", body)
        if not uri or not pose:
            continue
        values = [float(v) for v in pose.group(1).split()]
        plane_x = round(values[0] + WORLD_TO_MAP[0], 4)
        # yaw 0 means the code looks along +x, pi means -x.
        normal = 1 if abs(values[5]) < 1.0 else -1
        planes.setdefault((plane_x, normal), set()).add(round(values[1] + WORLD_TO_MAP[1], 4))
    return [(x, n, ys) for (x, n), ys in sorted(planes.items())]


def read_goal_tolerance():
    """The controller's active goal checker tolerances, as (xy, yaw)."""
    import yaml

    params = Path(get_package_share_directory("dc_demos")) / "params" / "qrcodes_nav.yaml"
    controller = yaml.safe_load(params.read_text())["controller_server"]["ros__parameters"]
    checker = controller[controller["goal_checker_plugins"][0]]
    return checker["xy_goal_tolerance"], checker["yaw_goal_tolerance"]


def check_map_offset(report):
    """WORLD_TO_MAP still describes the world and map the demo actually loads.

    Every number in check_qr_alignment is expressed in the map frame, so a wrong offset
    would make the whole check agree with itself and with nothing else. The demo states
    the same relation twice from opposite ends -- tb3_qrcodes.launch.py spawns the robot
    at a world pose, and qrcodes_nav.yaml hands AMCL the map pose of that same spot --
    so the two have to differ by exactly WORLD_TO_MAP.

    Args:
        report: collects pass/fail for the exit status.
    """
    import math

    import yaml

    launch = (
        Path(get_package_share_directory("dc_demos")) / "launch" / "tb3_qrcodes.launch.py"
    ).read_text()
    spawn = {}
    for axis in ("x", "y"):
        match = re.search(rf'"{axis}_pose", default="(-?[\d.]+)"', launch)
        if match:
            spawn[axis] = float(match.group(1))

    params = Path(get_package_share_directory("dc_demos")) / "params" / "qrcodes_nav.yaml"
    amcl = yaml.safe_load(params.read_text())["amcl"]["ros__parameters"]

    report.check(len(spawn) == 2, "found the demo's spawn pose", str(spawn))
    if len(spawn) != 2:
        return
    error = math.hypot(
        spawn["x"] + WORLD_TO_MAP[0] - amcl["initial_pose.x"],
        spawn["y"] + WORLD_TO_MAP[1] - amcl["initial_pose.y"],
    )
    report.check(
        error < 0.05,
        "map/world offset matches the demo's own spawn and initial poses",
        f"{round(error, 4)} m apart",
    )


def check_qr_alignment(report):
    """Every camera station keeps its QR code in frame across the whole goal tolerance.

    This is the check that was missing when #51 was filed. It relates the four things
    that have to agree and never did -- the camera's mounting in waffle.model, the
    stations in qrcodes_waypoint_follower.py, the pallet layout in qrcodes.world and the
    goal tolerance in qrcodes_nav.yaml -- and fails if a code can leave the frame at a
    pose Nav2 is allowed to stop at.

    Worst case is on the boundary of the tolerance circle, where shortening the standoff
    and sliding the code sideways trade off against each other; the closed form for that
    minimum is below. Yaw is the expensive term at these standoffs because it costs
    d*tan(yaw), so it grows with the very distance that buys the margin.

    Args:
        report: collects pass/fail for the exit status.
    """
    import math

    from dc_demos.qrcodes_waypoint_follower import camera_stations

    cameras = read_camera_poses()
    planes = read_code_planes()
    xy_tol, yaw_tol = read_goal_tolerance()
    report.check(bool(planes), "found QR-coded pallet faces", f"{len(planes)} faces")

    half_w_angle = DEFAULT_HFOV / 2.0
    # 1280x720, so the vertical half-angle follows from the horizontal one.
    half_h_angle = math.atan(math.tan(half_w_angle) * 720.0 / 1280.0)

    worst = None
    unseen = []
    for station_x, station_y, yaw in camera_stations():
        seen = False
        for side, (cx, cy, cz, cyaw) in cameras.items():
            # Camera position and viewing direction in the map frame.
            wx = station_x + cx * math.cos(yaw) - cy * math.sin(yaw)
            wy = station_y + cx * math.sin(yaw) + cy * math.cos(yaw)
            look = yaw + cyaw
            look_x = math.cos(look)
            if abs(look_x) < 0.9:  # not looking across the aisle at all
                continue
            for plane_x, normal, ys in planes:
                standoff = (plane_x - wx) * look_x
                # In front of this camera, facing it, and near enough to be its pallet.
                if standoff <= 0 or standoff > 1.5 or normal * look_x > 0:
                    continue
                target_y = min(ys, key=lambda y: abs(y - station_y))
                if abs(target_y - station_y) > 0.5:
                    continue
                seen = True
                # Worst case over the goal tolerance. The position error is a vector of
                # length up to xy_tol: its component across the aisle shortens the
                # standoff (a smaller frame), its component along the aisle slides the
                # code towards the edge, and the two trade off around the circle. With
                # slack = tan(hfov/2) - tan(yaw_tol), the horizontal budget at offset
                # angle phi is (d - r*cos phi)*slack - r*|sin phi|, whose minimum over
                # phi is d*slack - r*hypot(slack, 1). Vertically only the standoff
                # matters, so the worst case there is simply the nearest it may stop.
                slack = math.tan(half_w_angle) - math.tan(yaw_tol)
                margin_w = (
                    standoff * slack
                    - xy_tol * math.hypot(slack, 1.0)
                    - abs(target_y - wy)
                    - CODE_HALF_W
                )
                margin_h = (
                    (standoff - xy_tol) * math.tan(half_h_angle)
                    - abs(CODE_CENTRE_Z - cz)
                    - CODE_HALF_H
                )
                margin = min(margin_w, margin_h)
                if worst is None or margin < worst[0]:
                    worst = (margin, side, station_x, station_y, round(standoff, 3))
        if not seen:
            unseen.append((station_x, station_y))

    report.check(
        not unseen,
        "every camera station faces a QR-coded pallet",
        f"{len(unseen)} do not, first at {unseen[0]}" if unseen else "",
    )
    if worst is not None:
        margin, side, at_x, at_y, standoff = worst
        report.check(
            margin > 0,
            "QR codes stay in frame across the goal tolerance",
            f"worst margin {round(margin, 3)} m ({side} camera at ({at_x}, {at_y}), "
            f"standoff {standoff} m, tolerance {xy_tol} m / {yaw_tol} rad)",
        )


def main():
    report = Report()
    print("[lint] launch files")
    check_launch_loads(report)
    print("[lint] SDF")
    check_sdf(report)
    print("[lint] bridge config vs demo params")
    check_bridge_config(report)
    print("[lint] QR-code alignment")
    check_map_offset(report)
    check_qr_alignment(report)
    return report.finish()


if __name__ == "__main__":
    sys.exit(main())
