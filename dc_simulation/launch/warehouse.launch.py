import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Get the launch directory
    warehouse_dir = get_package_share_directory("dc_simulation")

    # I didn't find a function that returns the install folder
    # This line returns where the dc_simulation package is stored
    install_dir = re.match(r"(^.+)/dc_simulation/share/.+", warehouse_dir).group(1)

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")

    declare_simulator_cmd = DeclareLaunchArgument(
        "headless", default_value="False", description="Whether to execute gzclient)"
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(warehouse_dir, "worlds", "warehouse.world"),
        description="Full path to world model file to load",
    )

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        cwd=[install_dir],
        output="screen",
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(["not ", headless])),
        cmd=["gzclient"],
        cwd=[install_dir],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld
