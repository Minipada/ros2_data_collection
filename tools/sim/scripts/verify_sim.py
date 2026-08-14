#!/usr/bin/env python3
"""Assertions for the simulation smoke check (see tools/sim/scripts/run.sh).

Runs inside the container, inside the running simulation's ROS graph. Every check here
exists because a real bug got through CI without it — the whole demo layer is invisible
to `colcon build` and `colcon test`, since it lives in SDF, launch files, bridge configs
and params rather than in compiled code (#279).

Usage:
    verify_sim.py topics       # sensors and odometry carry usable data
    verify_sim.py cmd_vel      # publishing a Twist actually moves the robot
    verify_sim.py nav          # AMCL is localized against the static map
"""
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from tf2_msgs.msg import TFMessage

CAMERAS = [
    "/left_intel_realsense_r200_depth/image_raw",
    "/right_intel_realsense_r200_depth/image_raw",
]

# Generous: the warehouse world is heavy and CI runners are slow, so these bound "did it
# ever publish" rather than any rate. See dc_simulation/README.md for measured factors.
SETTLE_S = 120.0
DRIVE_S = 60.0


class Verifier(Node):
    def __init__(self):
        super().__init__("verify_sim")
        self.odom = None
        self.scan = None
        self.imu = None
        self.joints = None
        self.tf_frames = set()
        self.map = None
        self.images = {}
        self.create_subscription(Odometry, "/odom", self._set("odom"), 10)
        self.create_subscription(LaserScan, "/scan", self._set("scan"), 10)
        self.create_subscription(Imu, "/imu", self._set("imu"), 10)
        self.create_subscription(JointState, "/joint_states", self._set("joints"), 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        # map_server latches /map, so only a transient-local subscriber sees it.
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._set("map"),
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        for topic in CAMERAS:
            self.create_subscription(
                Image, topic, lambda m, t=topic: self.images.setdefault(t, m), 10
            )
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

    def _set(self, attr):
        def cb(msg):
            setattr(self, attr, msg)

        return cb

    def _on_tf(self, msg):
        for tf in msg.transforms:
            self.tf_frames.add((tf.header.frame_id, tf.child_frame_id))

    def spin(self, seconds):
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.2)

    def spin_until(self, predicate, seconds):
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                return True
        return predicate()


class Report:
    def __init__(self):
        self.failures = []

    def check(self, ok, name, detail=""):
        print(f"  {'PASS' if ok else 'FAIL'}  {name}{': ' + detail if detail else ''}")
        if not ok:
            self.failures.append(name)

    def finish(self):
        if self.failures:
            print(f"\nFAILED: {', '.join(self.failures)}")
            return 1
        print("\nall checks passed")
        return 0


def check_topics(node, report):
    """Every bridged topic carries data, and the sensor data is not degenerate.

    A silent topic here is the signature of three separate bugs this caught: a sensor
    with no <gz_frame_id> (dropped by every tf2 MessageFilter downstream), a bridge entry
    pointed at a gz topic nothing publishes on, and a robot that never spawned at all.

    Args:
        node: the Verifier spinning in the simulation's ROS graph.
        report: collects pass/fail for the exit status.
    """
    node.spin_until(
        lambda: all(x is not None for x in (node.odom, node.scan, node.imu, node.joints))
        and len(node.images) == len(CAMERAS)
        and node.tf_frames,
        SETTLE_S,
    )

    report.check(node.odom is not None, "/odom publishes")
    report.check(node.imu is not None, "/imu publishes")
    report.check(node.joints is not None, "/joint_states publishes")
    report.check(bool(node.tf_frames), "/tf publishes")

    if node.joints is not None:
        names = set(node.joints.name)
        report.check(
            {"wheel_left_joint", "wheel_right_joint"} <= names,
            "/joint_states carries both wheel joints",
            ",".join(sorted(names)),
        )

    if node.odom is not None:
        report.check(
            node.odom.header.frame_id == "odom" and node.odom.child_frame_id == "base_footprint",
            "/odom frames",
            f"{node.odom.header.frame_id} -> {node.odom.child_frame_id}",
        )

    scan = node.scan
    report.check(scan is not None, "/scan publishes")
    if scan is not None:
        # frame_id must be a real URDF link or AMCL and both costmaps silently drop it.
        report.check(scan.header.frame_id == "base_scan", "/scan frame_id", scan.header.frame_id)
        finite = [
            r for r in scan.ranges if math.isfinite(r) and scan.range_min < r < scan.range_max
        ]
        report.check(
            len(finite) > 10, "/scan has in-range returns", f"{len(finite)}/{len(scan.ranges)}"
        )
        # A wall of identical readings means the renderer produced nothing useful.
        distinct = len({round(r, 2) for r in finite})
        # round() rather than an f-string format spec: this flake8 misparses ":.2f"
        # inside an f-string as a dict colon and reports E231.
        spread = f", {round(min(finite), 2)}-{round(max(finite), 2)} m" if finite else ""
        report.check(distinct > 5, "/scan is not degenerate", f"{distinct} distinct ranges{spread}")

    for topic in CAMERAS:
        msg = node.images.get(topic)
        report.check(msg is not None, f"{topic} publishes")
        if msg is not None:
            report.check(
                any(msg.data),
                f"{topic} is not all-black",
                f"{msg.width}x{msg.height} {msg.encoding}",
            )
            report.check(
                msg.header.frame_id.startswith("camera_"),
                f"{topic} frame_id",
                msg.header.frame_id,
            )


def check_cmd_vel(node, report):
    """A Twist on /cmd_vel reaches DiffDrive and the robot moves.

    The one assertion that proves the robot is really in the world and really actuated,
    rather than merely present in a topic list.

    Args:
        node: the Verifier spinning in the simulation's ROS graph.
        report: collects pass/fail for the exit status.
    """
    if not node.spin_until(lambda: node.odom is not None, SETTLE_S):
        report.check(False, "/odom publishes before drive")
        return
    start = node.odom.pose.pose.position

    twist = Twist()
    twist.linear.x = 0.2
    end_ns = node.get_clock().now().nanoseconds + DRIVE_S * 1e9
    while node.get_clock().now().nanoseconds < end_ns:
        node.cmd_vel.publish(twist)
        node.spin(0.2)
    node.cmd_vel.publish(Twist())
    node.spin(5)

    end = node.odom.pose.pose.position
    moved = math.hypot(end.x - start.x, end.y - start.y)
    origin = f"({round(start.x, 2)}, {round(start.y, 2)})"
    report.check(moved > 0.05, "/cmd_vel moves the robot", f"{round(moved, 3)} m from {origin}")


def check_nav(node, report):
    """Nav2 is up and localized against the static map.

    Deliberately not asserting on /amcl_pose: AMCL publishes it on a filter update, and
    with update_min_d at 0.25 m a robot sitting still after bringup never produces one.
    map->odom is the assertion that holds for a stationary robot — AMCL re-broadcasts it
    continuously, and its presence is what every costmap and the whole planner chain
    actually depend on.

    Args:
        node: the Verifier spinning in the simulation's ROS graph.
        report: collects pass/fail for the exit status.
    """
    ok = node.spin_until(
        lambda: node.map is not None and ("map", "odom") in node.tf_frames, SETTLE_S
    )

    report.check(node.map is not None, "/map publishes")
    if node.map is not None:
        info = node.map.info
        report.check(
            info.width > 0 and info.height > 0,
            "/map has real dimensions",
            f"{info.width}x{info.height} @ {info.resolution} m/px",
        )

    report.check(
        ("map", "odom") in node.tf_frames,
        "AMCL broadcasts map->odom",
        f"{len(node.tf_frames)} transforms seen",
    )
    report.check(ok, "Nav2 localized")


CHECKS = {"topics": check_topics, "cmd_vel": check_cmd_vel, "nav": check_nav}


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in CHECKS:
        print(f"usage: {sys.argv[0]} {{{'|'.join(CHECKS)}}}", file=sys.stderr)
        return 2
    stage = sys.argv[1]
    rclpy.init()
    node = Verifier()
    report = Report()
    print(f"[verify_sim] {stage}")
    try:
        CHECKS[stage](node, report)
    finally:
        rclpy.shutdown()
    return report.finish()


if __name__ == "__main__":
    sys.exit(main())
