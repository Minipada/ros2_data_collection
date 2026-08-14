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


def main():
    report = Report()
    print("[lint] launch files")
    check_launch_loads(report)
    print("[lint] SDF")
    check_sdf(report)
    print("[lint] bridge config vs demo params")
    check_bridge_config(report)
    return report.finish()


if __name__ == "__main__":
    sys.exit(main())
