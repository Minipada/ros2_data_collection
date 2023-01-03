from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)  # Helper module
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion

"""
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
"""


def main():

    # Start the ROS 2 Python Client Library
    rclpy.init()

    # Launch the ROS 2 Navigation Stack
    navigator = BasicNavigator()

    # Wait for navigation to fully activate. Use this line if autostart is set to true.
    navigator.waitUntilNav2Active()

    # Set the robot's goal poses
    rows_x = [-19.6, -15.6, -11.85]
    z_orientation = [0.7071068967259818, -0.7023745033057341]  # Going up and down
    w_orientation = [0.7248122257854975, 0.7118075983761505]  # Going up and down
    space_pallets = 1.3
    space_rows = 2.5 - space_pallets  # since we count the space_pallets in calculation
    pallet_per_row = 10
    rows_count = 2
    rows_y0 = -10.39

    row_1 = [
        PoseStamped(
            header=Header(frame_id="map", stamp=navigator.get_clock().now().to_msg()),
            pose=Pose(
                position=Point(
                    x=rows_x[0],
                    y=rows_y0
                    + space_pallets * index_pall
                    + space_rows * (index_pall // pallet_per_row),
                    z=0.0,
                ),
                orientation=Quaternion(
                    x=0.0, y=0.0, z=z_orientation[0], w=w_orientation[0]
                ),
            ),
        )
        for index_pall in range(0, rows_count * pallet_per_row)
    ]

    row_2 = [
        PoseStamped(
            header=Header(frame_id="map", stamp=navigator.get_clock().now().to_msg()),
            pose=Pose(
                position=Point(
                    x=rows_x[1],
                    y=rows_y0
                    + (space_pallets) * index_pall
                    + space_rows * (index_pall // pallet_per_row),
                    z=0.0,
                ),
                orientation=Quaternion(
                    x=0.0, y=0.0, z=z_orientation[1], w=w_orientation[1]
                ),
            ),
        )
        for index_pall in list(reversed(range(0, rows_count * pallet_per_row)))
    ]

    row_3 = [
        PoseStamped(
            header=Header(frame_id="map", stamp=navigator.get_clock().now().to_msg()),
            pose=Pose(
                position=Point(
                    x=rows_x[2],
                    y=rows_y0
                    + space_pallets * index_pall
                    + space_rows * (index_pall // pallet_per_row),
                    z=0.0,
                ),
                orientation=Quaternion(
                    x=0.0, y=0.0, z=z_orientation[0], w=w_orientation[0]
                ),
            ),
        )
        for index_pall in range(0, rows_count * pallet_per_row)
    ]
    goal_poses = row_1 + row_2 + row_3

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
                navigator.cancelNav()

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

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == "__main__":
    main()
