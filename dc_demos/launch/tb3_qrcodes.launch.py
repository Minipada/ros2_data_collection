"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    demos_dir = get_package_share_directory("dc_demos")
    sim_dir = get_package_share_directory("dc_simulation")
    dc_description_dir = get_package_share_directory("dc_description")
    dc_bringup_dir = get_package_share_directory("dc_bringup")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # Data Collection
    dc_params_file = LaunchConfiguration("dc_params_file")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_dc = LaunchConfiguration("use_dc")

    # Nav2
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    map_yaml_file = LaunchConfiguration("map")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_simulator = LaunchConfiguration("use_simulator")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")  # Should always be false
    use_rviz = LaunchConfiguration("use_rviz")
    slam = LaunchConfiguration("slam")
    use_namespace = LaunchConfiguration("use_namespace")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose", default="-16.679400")
    y_pose = LaunchConfiguration("y_pose", default="-15.300200")
    z_pose = LaunchConfiguration("z_pose", default="0.01")
    roll = LaunchConfiguration("roll", default="0.00")
    pitch = LaunchConfiguration("pitch", default="0.00")
    yaw = LaunchConfiguration("yaw", default="1.570796")

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(sim_dir, "maps", "qrcodes.yaml"),
        description="Full path to map file to load",
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="False",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "headless", default_value="True", description="Whether to execute gzclient"
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="False",
        description="Whether to start the robot state publisher.",
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(sim_dir, "rviz", "qrcodes.rviz"),
        description="Full path to the RVIZ config file to use",
    )
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(sim_dir, "worlds", "qrcodes.world"),
        description="Full path to world model file to load",
    )
    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name", default_value="turtlebot3_waffle", description="name of the robot"
    )
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        "robot_sdf",
        default_value=os.path.join(sim_dir, "worlds", "waffle.model"),
        description="Full path to robot sdf file to spawn the robot in gazebo",
    )
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(demos_dir, "params", "qrcodes_nav.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    # DC
    declare_use_dc = DeclareLaunchArgument(
        "use_dc", default_value="True", description="Start Data Collection"
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_dc_params_file_cmd = DeclareLaunchArgument(
        "dc_params_file",
        default_value=os.path.join(demos_dir, "params", "qrcodes_stdout.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the dc stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="dc_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator",
        default_value="True",
        description="Whether to start the simulator",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    # Use custom URDF with camera
    urdf = os.path.join(dc_description_dir, "urdf", "turtlebot3_waffle_qrcodes.xacro")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": Command(["xacro ", urdf]),
            }
        ],
        remappings=remappings,
        arguments=["urdf", urdf],
    )

    dc_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dc_bringup_dir, "launch", "dc_bringup.launch.py")
        ),
        condition=IfCondition(use_dc),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "dc_params_file": dc_params_file,
            "use_composition": use_composition,
            "container_name": container_name,
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "tb3_simulation_launch.py")
        ),
        launch_arguments={
            "map": map_yaml_file,
            "params_file": nav2_params_file,
            "namespace": namespace,
            "use_namespace": use_namespace,
            "slam": slam,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "log_level": log_level,
            "rviz_config_file": rviz_config_file,
            "use_simulator": use_simulator,
            "use_rviz": use_rviz,
            "headless": headless,
            "world": world,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "z_pose": z_pose,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "use_robot_state_pub": use_robot_state_pub,
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_dc)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_dc_params_file_cmd)
    ld.add_action(declare_log_level_cmd)

    # Declare the launch options
    ld.add_action(dc_bringup_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(nav2_bringup_cmd)

    return ld
