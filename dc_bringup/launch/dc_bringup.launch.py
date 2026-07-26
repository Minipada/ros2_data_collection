import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def start_collection_after_gate(collection_actions):
    """Handler for the bridge_ready_gate's exit (ADR-0006 startup ordering).

    Gate exited 0 (Bridge reported ready): start the lifecycle manager, which
    configures and activates the collection nodes. Gate exited non-zero (the
    Bridge never became ready before the gate's deadline): shut the whole
    launch down loudly rather than leave a half-started pipeline running.
    """

    def on_gate_exit(event, context):
        if event.returncode == 0:
            return [
                LogInfo(msg="dc_bridge reports ready; activating collection nodes."),
                *collection_actions,
            ]
        return [
            LogInfo(
                msg="dc_bridge never became ready "
                f"(bridge_ready_gate exited {event.returncode}); shutting down."
            ),
            Shutdown(reason="dc_bridge never became ready"),
        ]

    return on_gate_exit


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("dc_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    dc_params_file = LaunchConfiguration("dc_params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    detection_barcodes_service = LaunchConfiguration("detection_barcodes_service")
    draw_img_service = LaunchConfiguration("draw_img_service")
    save_img_service = LaunchConfiguration("save_img_service")
    group_node = LaunchConfiguration("group_node")

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"autostart": autostart}

    configured_params = RewrittenYaml(
        source_file=dc_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "dc_params_file",
        default_value=os.path.join(bringup_dir, "params", "dc_params.yaml"),
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

    declare_detection_barcodes_service = DeclareLaunchArgument(
        "detection_barcodes_service",
        default_value="False",
        description="Start barcode detection service",
    )
    declare_draw_img_service = DeclareLaunchArgument(
        "draw_img_service", default_value="False", description="Start draw image service"
    )
    declare_save_img_service = DeclareLaunchArgument(
        "save_img_service", default_value="False", description="Start save image service"
    )
    declare_group_node = DeclareLaunchArgument(
        "group_node", default_value="False", description="Start group_node"
    )

    # ADR-0006 deterministic startup: the Bridge (which spawns and supervises
    # Vector) comes up as a plain node first; a readiness gate process blocks on
    # its ~/ready service; only when the gate exits successfully does the
    # lifecycle manager start and activate the collection nodes. No Record can
    # be emitted before the pipeline can accept it. The gate and the deferred
    # lifecycle manager exist once per composition branch because a launch
    # action instance cannot be shared between groups.
    lifecycle_manager_params = [
        {
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": [
                "measurement_server",
            ],
            "transitions": ["configure", "activate"],
            "bond_timeout": 10.0,
        },
    ]

    bridge_ready_gate = Node(
        package="dc_bringup",
        executable="bridge_ready_gate",
        name="bridge_ready_gate",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[configured_params],
    )

    lifecycle_manager_node = Node(
        package="dc_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_dc",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        arguments=["--ros-args", "--log-level", log_level],
        parameters=lifecycle_manager_params,
    )

    bridge_ready_gate_composed = Node(
        package="dc_bringup",
        executable="bridge_ready_gate",
        name="bridge_ready_gate",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[configured_params],
    )

    load_lifecycle_manager_composed = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package="dc_lifecycle_manager",
                plugin="dc_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_dc",
                parameters=lifecycle_manager_params,
            ),
        ],
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("dc_services"),
                        "launch",
                        "dc_save_image.launch.py",
                    )
                ),
                condition=IfCondition(save_img_service),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("dc_services"),
                        "launch",
                        "dc_draw_image.launch.py",
                    )
                ),
                condition=IfCondition(draw_img_service),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("dc_services"),
                        "launch",
                        "dc_detection_barcodes.launch.py",
                    )
                ),
                condition=IfCondition(detection_barcodes_service),
            ),
            Node(
                package="dc_measurements",
                executable="measurement_server",
                name="measurement_server",
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=IfCondition(group_node),
                package="dc_group",
                executable="group_server",
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            # dc_bridge replaces the old destination_server/dc_destinations pluginlib layer
            # (removed per ADR-0001/0003, #250), forwarding Records to the Vector shipper it
            # supervises internally (ADRs 0001/0006/0007). It is a plain node outside the
            # lifecycle manager, so it isn't in lifecycle_manager_dc's node_names; launch
            # respawn supervises it unconditionally (ADR-0006), independent of use_respawn.
            Node(
                package="dc_bridge",
                executable="dc_bridge",
                name="dc_bridge",
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
                respawn=True,
                respawn_delay=2.0,
                parameters=[configured_params],
            ),
            bridge_ready_gate,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=bridge_ready_gate,
                    on_exit=start_collection_after_gate([lifecycle_manager_node]),
                )
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("dc_services"),
                        "launch",
                        "dc_save_image.launch.py",
                    )
                ),
                condition=IfCondition(save_img_service),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("dc_services"),
                        "launch",
                        "dc_draw_image.launch.py",
                    )
                ),
                condition=IfCondition(draw_img_service),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("dc_services"),
                        "launch",
                        "dc_detection_barcodes.launch.py",
                    )
                ),
                condition=IfCondition(detection_barcodes_service),
            ),
            Node(
                condition=IfCondition(use_composition),
                name=container_name,
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            Node(
                condition=IfCondition(group_node),
                package="dc_group",
                executable="group_server",
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            # dc_bridge replaces the old destination_server/dc_destinations pluginlib layer
            # (removed per ADR-0001/0003, #250). It isn't built/registered as an
            # rclcpp_components plugin, so it always runs as a plain node (ADR-0006), even
            # when the rest of the stack is composed; launch respawn supervises it
            # unconditionally, independent of use_respawn.
            Node(
                package="dc_bridge",
                executable="dc_bridge",
                name="dc_bridge",
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
                respawn=True,
                respawn_delay=2.0,
                parameters=[configured_params],
            ),
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    ComposableNode(
                        package="dc_measurements",
                        plugin="measurement_server::MeasurementServer",
                        name="measurement_server",
                        parameters=[configured_params],
                    ),
                ],
            ),
            bridge_ready_gate_composed,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=bridge_ready_gate_composed,
                    on_exit=start_collection_after_gate([load_lifecycle_manager_composed]),
                )
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_detection_barcodes_service)
    ld.add_action(declare_draw_img_service)
    ld.add_action(declare_save_img_service)
    ld.add_action(declare_group_node)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
