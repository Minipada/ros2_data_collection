#!/usr/bin/env python3
"""Follow waypoints using the ROS 2 Navigation Stack (Nav2)."""

import math
import sys

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from std_msgs.msg import Header

# Set the robot's goal poses.
#
# Every number here is measured against dc_simulation/worlds/qrcodes.world and
# dc_simulation/maps/qrcodes.{pgm,yaml}; the map frame is the world frame shifted
# by (-7.300, +1.425) m (#51). In the map frame the four QR-coded pallet faces
# stand at x = -18.608, -16.298, -14.804 and -12.498, and each row of codes runs
# along y from -10.475 in steps of 1.3 m with a 2.5 m gap after the tenth.
#
# ROWS_X therefore stands the robot ~0.97 m off the codes in the two open aisles
# and mid-way between the two walls of the closed one. It used to read
# [-19.6, -15.6, -11.85]: 0.65 m off the codes in aisle 3 and 0.05 m off centre
# in aisle 2, which left the 60-degree lens no room for the goal tolerance. See
# qrcodes_nav.yaml's general_goal_checker for the arithmetic, and
# tools/sim/scripts/lint_launch_files.py for the check that holds it.
ROWS_X = [-19.6, -15.55, -11.5]
# Exactly +/- 90 degrees, so the cameras look square at the pallet faces. These
# used to be (z=0.7071068967, w=0.7248122258) and (z=-0.7023745033,
# w=0.7118075984) -- norms 1.0126 and 1.0000, i.e. headings of 88.6 and -89.2
# degrees rather than +/-90. The same transcription error in the initial pose
# below was fixed in #279; these sixty were missed.
ROW_YAW = [math.pi / 2, -math.pi / 2]  # Going up and down
SPACE_PALLETS = 1.3
SPACE_ROWS = 2.5 - SPACE_PALLETS  # since we count SPACE_PALLETS in the calculation
PALLET_PER_ROW = 10
ROWS_COUNT = 2
# Centre of the first pallet in the map frame; was -10.45, 25 mm off.
ROWS_Y0 = -10.475


def camera_stations():
    """The pose the robot takes in front of each QR-coded pallet, in the map frame.

    One entry per pallet, in visiting order: up aisle 1, down aisle 2, up aisle 3.

    Returns:
        A list of (x, y, yaw) tuples, metres and radians in the map frame.
    """
    stations = []
    for row, (row_x, yaw) in enumerate(
        zip(ROWS_X, [ROW_YAW[0], ROW_YAW[1], ROW_YAW[0]], strict=True)
    ):
        indices = range(ROWS_COUNT * PALLET_PER_ROW)
        # The middle row is driven the other way, so the robot does not have to
        # cross the whole warehouse between aisles.
        for index in reversed(indices) if row == 1 else indices:
            offset = SPACE_PALLETS * index + SPACE_ROWS * (index // PALLET_PER_ROW)
            stations.append((row_x, ROWS_Y0 + offset, yaw))
    return stations


def main():
    """Wait Navigation is active and go through points, in front of pallets with QRcodes."""
    # Start the ROS 2 Python Client Library
    rclpy.init()

    # Launch the ROS 2 Navigation Stack
    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # tb3_qrcodes.launch.py's spawn pose in the map frame; the same value
    # qrcodes_nav.yaml's amcl initial_pose.* carries. See #51 for the 69 mm
    # correction and for where the map/world offset comes from.
    initial_pose.pose.position.x = -23.9794
    initial_pose.pose.position.y = -13.8752
    # yaw 1.570796, as in the launch file and the params.
    # The value transcribed here used to be (z=0.7071068967, w=0.7248122258),
    # whose norm is 1.012 -- AMCL answers a non-unit quaternion with "Received
    # initialpose message is malformed. Rejecting.", so this call did nothing and
    # the demo silently relied on amcl's set_initial_pose instead (#279).
    initial_pose.pose.orientation.z = 0.7071067811865476
    initial_pose.pose.orientation.w = 0.7071067811865476
    navigator.setInitialPose(initial_pose)
    # Wait for navigation to fully activate. Use this line if autostart is set to true.
    navigator.waitUntilNav2Active()

    goal_poses = [
        PoseStamped(
            header=Header(frame_id="map", stamp=navigator.get_clock().now().to_msg()),
            pose=Pose(
                position=Point(x=station_x, y=station_y, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)),
            ),
        )
        for station_x, station_y, yaw in camera_stations()
    ]

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
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

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=100000000.0):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal succeeded!")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled!")
    elif result == TaskResult.FAILED:
        print("Goal failed!")
    else:
        print("Goal has an invalid return status!")

    # The pass is over once every waypoint has been visited. This used to sit
    # inside a `while rclpy.ok():` loop, which re-entered with the task already
    # complete and reprinted the result forever instead of exiting (#279).
    rclpy.shutdown()
    return 0 if result == TaskResult.SUCCEEDED else 1


if __name__ == "__main__":
    sys.exit(main())
