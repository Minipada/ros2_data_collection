import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory("dc_simulation")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    # waffle.model's visuals are the TurtleBot3 meshes, which no longer ship in
    # a Jazzy package of their own (turtlebot3_gazebo has no Jazzy release);
    # nav2_minimal_tb3_sim is where they live now, under model://turtlebot3_model.
    tb3_models_dir = os.path.join(get_package_share_directory("nav2_minimal_tb3_sim"), "models")

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    robot_sdf = LaunchConfiguration("robot_sdf")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    declare_simulator_cmd = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Whether to run gz-sim server-only (no GUI)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        # There is no "warehouse.world" -- the actual warehouse (with its
        # shelf/prop dressing and QR-coded pallets) is qrcodes.world.
        default_value=os.path.join(sim_dir, "worlds", "qrcodes.world"),
        description="Full path to world sdf file to load",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name", default_value="turtlebot3_waffle", description="name of the robot"
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        "robot_sdf",
        default_value=os.path.join(sim_dir, "worlds", "waffle.model"),
        description="Full path to robot sdf file to spawn the robot in gz-sim",
    )

    declare_x_pose_cmd = DeclareLaunchArgument(
        "x_pose", default_value="-7.3", description="Initial robot x pose"
    )
    declare_y_pose_cmd = DeclareLaunchArgument(
        "y_pose", default_value="1.37", description="Initial robot y pose"
    )
    declare_z_pose_cmd = DeclareLaunchArgument(
        "z_pose", default_value="0.01", description="Initial robot z pose"
    )
    declare_roll_cmd = DeclareLaunchArgument(
        "roll", default_value="0.00", description="Initial robot roll"
    )
    declare_pitch_cmd = DeclareLaunchArgument(
        "pitch", default_value="0.00", description="Initial robot pitch"
    )
    declare_yaw_cmd = DeclareLaunchArgument(
        "yaw", default_value="0.00", description="Initial robot yaw"
    )

    # gz-sim itself (server, with GUI unless headless)
    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": PythonExpression(
                ["'", world, "' + (' -s' if '", headless, "' == 'True' else '') + ' -r'"]
            ),
        }.items(),
    )

    # Spawn the TurtleBot3-Waffle into the running world
    spawn_robot_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            robot_sdf,
            "-name",
            robot_name,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
            "-R",
            roll,
            "-P",
            pitch,
            "-Y",
            yaw,
        ],
    )

    # Bridge the robot's gz-transport topics (cmd_vel, odom, tf, imu, scan,
    # joint_states, cameras) to ROS -- see config/warehouse_bridge.yaml.
    bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file": os.path.join(sim_dir, "config", "warehouse_bridge.yaml"),
                # The bridge is the node that publishes /clock, so it has to run
                # on the wall clock itself; everything downstream of it does not.
                "use_sim_time": False,
            }
        ],
    )

    set_tb3_model_path_cmd = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", tb3_models_dir)

    ld = LaunchDescription()

    ld.add_action(set_tb3_model_path_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(gz_sim_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(bridge_cmd)

    return ld
