"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    demos_dir = get_package_share_directory("dc_demos")
    dc_description_dir = get_package_share_directory("dc_description")
    dc_bringup_dir = get_package_share_directory("dc_bringup")

    # Data Collection
    dc_params_file = LaunchConfiguration("dc_params_file")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    group_node = LaunchConfiguration("group_node")
    detection_barcodes_service = LaunchConfiguration("detection_barcodes_service")
    draw_img_service = LaunchConfiguration("draw_img_service")
    save_img_service = LaunchConfiguration("save_img_service")

    # DC
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_dc_params_file_cmd = DeclareLaunchArgument(
        "dc_params_file",
        default_value=os.path.join(demos_dir, "params", "qrcodes_minio_pgsql.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the dc stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
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

    declare_detection_barcodes_service = DeclareLaunchArgument(
        "detection_barcodes_service",
        default_value="True",
        description="Start barcode detection service",
    )
    declare_draw_img_service = DeclareLaunchArgument(
        "draw_img_service", default_value="True", description="Start draw image service"
    )
    declare_save_img_service = DeclareLaunchArgument(
        "save_img_service", default_value="True", description="Start save image service"
    )
    declare_group_node = DeclareLaunchArgument(
        "group_node", default_value="True", description="Start group_node"
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
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "dc_params_file": dc_params_file,
            "use_composition": use_composition,
            "container_name": container_name,
            "use_respawn": use_respawn,
            "log_level": log_level,
            "group_node": group_node,
            "detection_barcodes_service": detection_barcodes_service,
            "save_img_service": save_img_service,
            "draw_img_service": draw_img_service,
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_dc_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_detection_barcodes_service)
    ld.add_action(declare_draw_img_service)
    ld.add_action(declare_save_img_service)
    ld.add_action(declare_group_node)

    # Declare the launch options
    ld.add_action(dc_bringup_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
