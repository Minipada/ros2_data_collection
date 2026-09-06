#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Follow waypoints in dc_simulation/worlds/qrcodes_small.world using Nav2.

The small-world counterpart to qrcodes_waypoint_follower.py (#473's sim-job flakiness
fix) -- same structure, but against the 4-station room dc_simulation/tools/gen_small_
world.py generates rather than the real 60-station warehouse. Kept as its own script
rather than a parameterized branch of the real one: the two demos' station lists,
spawn poses and world/map files are independent fixtures with no shared state to keep
in sync beyond the numbers below, which must match gen_small_world.py's own STATIONS
and SPAWN constants.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from std_msgs.msg import Header

# Must match dc_simulation/tools/gen_small_world.py's STATIONS/station_positions() and
# SPAWN -- world frame == map frame in this fixture, so these are the same numbers the
# launch invocation passes as x_pose/y_pose/yaw.
STATIONS = [
    (-1.07, -3.0, 1.570796),
    (-1.07, -1.7, 1.570796),
    (2.53, 1.7, 1.570796),
    (2.53, 3.0, 1.570796),
]
SPAWN_X = -1.07
SPAWN_Y = -4.3
SPAWN_YAW = 1.570796


def main():
    """Wait for Navigation to activate, then visit each QR-coded pallet station."""
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = SPAWN_X
    initial_pose.pose.position.y = SPAWN_Y
    initial_pose.pose.orientation.z = math.sin(SPAWN_YAW / 2.0)
    initial_pose.pose.orientation.w = math.cos(SPAWN_YAW / 2.0)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    goal_poses = [
        PoseStamped(
            header=Header(frame_id="map", stamp=navigator.get_clock().now().to_msg()),
            pose=Pose(
                position=Point(x=station_x, y=station_y, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)),
            ),
        )
        for station_x, station_y, yaw in STATIONS
    ]

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                "Executing current waypoint: "
                + str(feedback.current_waypoint + 1)
                + "/"
                + str(len(goal_poses))
            )
            now = navigator.get_clock().now()
            if now - nav_start > Duration(seconds=100000000.0):
                navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal succeeded!")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled!")
    elif result == TaskResult.FAILED:
        print("Goal failed!")
    else:
        print("Goal has an invalid return status!")

    rclpy.shutdown()
    return 0 if result == TaskResult.SUCCEEDED else 1


if __name__ == "__main__":
    sys.exit(main())
