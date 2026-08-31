# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
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


def _topic_to_dc_tag_route(topic: str) -> str:
    """Convert a ROS topic name to its public `dc.<tag>` Shipper route (ADR-0003).

    Args:
        topic: A ROS topic name, e.g. `/dc/measurement/cpu`.

    Returns:
        The route name a passthrough sink consumes, e.g. `dc.dc.measurement.cpu`.
    """
    tag = topic.lstrip("/").replace("/", ".")
    return f"dc.{tag}"


def build_uploader_action(raw_params):
    """Build the `dc_uploader` process (#446) for the `dc_bridge:` block's files Destination.

    The Uploader used to run on a worker thread inside `dc_bridge`; it is now a
    standalone process (docs/adr/0014-uploader-runs-as-its-own-process.md),
    configured entirely by `DC_UPLOADER_*` environment variables
    (`dc_bridge/uploader/process_config.hpp`) rather than ROS parameters, so it has
    no ROS dependency. This function is what still makes it *configure* like one
    block in the params file: it reads the `dc_bridge:` block's single `receives:
    files` Destination (if any) plus `uploader.data_dir`/`vector_forward_host`/
    `vector_forward_port`/`files.*`, and translates them into that process's
    environment — the one place a files Destination's settings cross from ROS
    params into `DC_UPLOADER_*` env vars.

    Started the same way `dc_mcap_writer` is (see build_bridge_and_mcap_actions):
    `ExecuteProcess` running the installed binary directly, not `ros2 run` (which
    would not forward signals to it — see that function's docstring), with
    `respawn=True` so an Uploader crash restarts it freely without taking the
    Bridge down (#446's "killing the Uploader does not affect Record collection").

    Args:
        raw_params: The parsed params file (same dict build_bridge_and_mcap_actions
            already loaded from `dc_params_file`).

    Returns:
        A list with one `ExecuteProcess` action if a `receives: files` Destination
        is configured, otherwise an empty list.
    """
    dc_bridge_params = (raw_params.get("dc_bridge") or {}).get("ros__parameters") or {}
    destination_names = dc_bridge_params.get("destinations", [])

    def _receives(destination_name):
        return (dc_bridge_params.get(destination_name) or {}).get("receives")

    files_destination_names = [name for name in destination_names if _receives(name) == "files"]
    if not files_destination_names:
        return []
    if len(files_destination_names) > 1:
        # The split (#446) carries one object-storage endpoint's worth of
        # DC_UPLOADER_* env vars; supporting more is future work, not silent data
        # loss for one of them.
        raise RuntimeError(
            "dc_uploader launch wiring supports at most one `receives: files` destination per "
            f"dc_bridge params file; found {files_destination_names}"
        )
    name = files_destination_names[0]
    dest = dc_bridge_params.get(name) or {}

    shipper_params = dc_bridge_params.get("shipper") or {}
    shipper_data_dir = os.path.expanduser(
        os.path.expandvars(shipper_params.get("data_dir", "$HOME/.dc/buffer"))
    )
    uploader_params = dc_bridge_params.get("uploader") or {}
    uploader_data_dir = os.path.expanduser(
        os.path.expandvars(uploader_params.get("data_dir", shipper_data_dir))
    )
    files_params = dc_bridge_params.get("files") or {}
    retention_params = files_params.get("retention") or {}
    delete_when_sent = files_params.get("delete_when_sent", False)

    uploader_env = {
        "DC_UPLOADER_STORAGE_NAME": str(name),
        # Mirrors bridge_node.cpp's own `<uploader.data_dir>/queue/upload` and
        # `<uploader.data_dir>/uploader` — the two processes must agree on these
        # paths without either hard-coding the other's binary.
        "DC_UPLOADER_QUEUE_DIR": os.path.join(uploader_data_dir, "queue", "upload"),
        "DC_UPLOADER_STATE_DIR": os.path.join(uploader_data_dir, "uploader"),
        "DC_UPLOADER_SHIPPER_HOST": str(dc_bridge_params.get("vector_forward_host", "127.0.0.1")),
        "DC_UPLOADER_SHIPPER_PORT": str(dc_bridge_params.get("vector_forward_port", 24224)),
        "DC_UPLOADER_S3_BUCKET": str(dest.get("bucket", "")),
        "DC_UPLOADER_DELETE_WHEN_SENT": "true" if delete_when_sent else "false",
    }
    if dest.get("region"):
        uploader_env["DC_UPLOADER_S3_REGION"] = str(dest["region"])
    if dest.get("endpoint"):
        uploader_env["DC_UPLOADER_S3_ENDPOINT"] = str(dest["endpoint"])
    if dest.get("key_prefix"):
        uploader_env["DC_UPLOADER_S3_KEY_PREFIX"] = str(dest["key_prefix"])
    if dest.get("access_key_id") and dest.get("secret_access_key"):
        uploader_env["DC_UPLOADER_S3_ACCESS_KEY_ID"] = str(dest["access_key_id"])
        # Secrets support $VAR-style env references (ADR-0003), same as
        # declare_destination's C++ side expands `secret_access_key`/`password`.
        secret = os.path.expandvars(str(dest["secret_access_key"]))
        uploader_env["DC_UPLOADER_S3_SECRET_ACCESS_KEY"] = secret
    force_path_style = dest.get("force_path_style")
    if force_path_style is not None:
        uploader_env["DC_UPLOADER_S3_FORCE_PATH_STYLE"] = "true" if force_path_style else "false"
    max_bytes = retention_params.get("max_bytes")
    if max_bytes:
        uploader_env["DC_UPLOADER_RETENTION_MAX_BYTES"] = str(max_bytes)
    max_age_days = retention_params.get("max_age_days")
    if max_age_days:
        uploader_env["DC_UPLOADER_RETENTION_MAX_AGE_DAYS"] = str(max_age_days)
    if files_params.get("ffprobe_binary"):
        uploader_env["DC_UPLOADER_FFPROBE_BINARY"] = str(files_params["ffprobe_binary"])
    multipart_threshold = files_params.get("multipart_threshold_bytes")
    if multipart_threshold:
        uploader_env["DC_UPLOADER_MULTIPART_THRESHOLD_BYTES"] = str(multipart_threshold)
    multipart_part_size = files_params.get("multipart_part_size_bytes")
    if multipart_part_size:
        uploader_env["DC_UPLOADER_MULTIPART_PART_SIZE_BYTES"] = str(multipart_part_size)

    thumbnails_params = files_params.get("thumbnails") or {}
    if thumbnails_params.get("enabled", False):
        uploader_env["DC_UPLOADER_THUMBNAILS_ENABLED"] = "true"
    max_dimension = thumbnails_params.get("max_dimension")
    if max_dimension:
        uploader_env["DC_UPLOADER_THUMBNAIL_MAX_DIMENSION"] = str(max_dimension)
    if thumbnails_params.get("ffmpeg_binary"):
        uploader_env["DC_UPLOADER_FFMPEG_BINARY"] = str(thumbnails_params["ffmpeg_binary"])

    uploader_binary = os.path.join(
        get_package_prefix("dc_bridge"), "lib", "dc_bridge", "dc_uploader"
    )
    return [
        ExecuteProcess(
            cmd=[uploader_binary],
            name="dc_uploader",
            output={"stdout": "screen", "stderr": "screen"},
            additional_env=uploader_env,
            respawn=True,
            respawn_delay=2.0,
        )
    ]


def build_bridge_and_mcap_actions(configured_params):
    """Build the `dc_bridge` Node, plus `dc_mcap_writer` if the params file enables it.

    `dc_mcap_writer` (ADR-0009, #210) is not a Vector sink — Vector has none for
    MCAP — and not a blessed Destination `dc_bridge` itself knows about (ADR-0003
    reserves that C++ validation path for `postgres`/`s3`/`file`/`console`; teaching it
    a fifth type would mean touching `dc_bridge`, which ADR-0009 explicitly avoids). To
    still make it *configure* like any other Destination — one block in the same params
    file, no hand-authored Vector TOML, no separately-started process — this function
    reads a `dc_mcap_writer:` top-level block (sibling to `dc_bridge:`, not nested
    inside it) at launch time, and if `enabled: true`:

    1. Renders the ADR-0009 passthrough sink (a Vector `socket` sink) from its `inputs`
       to a generated TOML file, and merges that file into `dc_bridge`'s own
       `custom_config_files` (on top of whatever the params file already lists there —
       a second parameters-list entry *overrides* a list-valued parameter, so the merge
       has to happen here, in Python, not by relying on launch_ros to combine them).
    2. Starts `dc_mcap_writer` itself as a plain `ExecuteProcess`, not `ros2 run`: `ros2
       run` spawns its target as a child of its own process and does not forward
       signals to it, so a `respawn`/shutdown-triggering `kill` on what `ros2 run`
       returns never reaches the actual writer (see `tools/e2e/scripts/entrypoint.sh`
       for the same pattern). `ExecuteProcess` runs `python3 -m dc_mcap_writer.cli`
       directly instead.

    When the block is absent or `enabled` is false or missing (every existing params
    file, unchanged), this returns exactly what the static `Node(...)` it replaces used
    to: `dc_bridge` alone, with `configured_params` as its only parameters entry.

    Args:
        configured_params: The `RewrittenYaml` substitution both branches of
            `generate_launch_description` already build from `dc_params_file`.

    Returns:
        An `OpaqueFunction` action that resolves `dc_params_file` at launch time and
        returns the `dc_bridge` Node plus, when enabled, the `dc_mcap_writer` process.
    """

    def _build(context, *args, **kwargs):
        params_file_path = LaunchConfiguration("dc_params_file").perform(context)
        try:
            with open(params_file_path) as f:
                raw_params = yaml.safe_load(f) or {}
        except OSError:
            raw_params = {}

        mcap_params = (raw_params.get("dc_mcap_writer") or {}).get("ros__parameters") or {}
        bridge_parameters = [configured_params]
        extra_actions = []

        if mcap_params.get("enabled", False):
            inputs = mcap_params.get("inputs", [])
            host = mcap_params.get("host", "127.0.0.1")
            port = mcap_params.get("port", 9191)
            output_dir = os.path.expanduser(
                os.path.expandvars(mcap_params.get("output_dir", "$HOME/.dc/mcap"))
            )
            max_bytes = mcap_params.get("max_bytes", 128 * 1024 * 1024)
            max_duration_secs = mcap_params.get("max_duration_secs", 300)
            prefix = mcap_params.get("prefix", "records")

            routes = ", ".join(f'"{_topic_to_dc_tag_route(topic)}"' for topic in inputs)
            listen_address = str(host) + ":" + str(port)
            sink_toml = (
                f"# Generated by dc_bringup.launch.py from the dc_mcap_writer: block in\n"
                f"# {params_file_path} (ADR-0009, #210) — regenerated every launch, not\n"
                f"# meant to be hand-edited.\n"
                f"[sinks.dc_mcap_writer]\n"
                f'type = "socket"\n'
                f"inputs = [{routes}]\n"
                f'mode = "tcp"\n'
                f'address = "{listen_address}"\n'
                f'encoding.codec = "json"\n'
                f'framing.method = "newline_delimited"\n'
                f"\n"
                f"[sinks.dc_mcap_writer.buffer]\n"
                f'type = "disk"\n'
                f"max_size = 268435488\n"
            )
            generated_dir = os.path.expanduser("~/.dc")
            os.makedirs(generated_dir, exist_ok=True)
            generated_sink_path = os.path.join(generated_dir, "generated_mcap_sink.toml")
            with open(generated_sink_path, "w") as f:
                f.write(sink_toml)

            dc_bridge_params = (raw_params.get("dc_bridge") or {}).get("ros__parameters") or {}
            merged_custom_config_files = list(dc_bridge_params.get("custom_config_files", [])) + [
                generated_sink_path
            ]
            bridge_parameters = [
                configured_params,
                {"custom_config_files": merged_custom_config_files},
            ]

            extra_actions.append(
                ExecuteProcess(
                    cmd=[
                        "python3",
                        "-m",
                        "dc_mcap_writer.cli",
                        "--output-dir",
                        output_dir,
                        "--host",
                        str(host),
                        "--port",
                        str(port),
                        "--max-bytes",
                        str(max_bytes),
                        "--max-duration-secs",
                        str(max_duration_secs),
                        "--prefix",
                        str(prefix),
                    ],
                    name="dc_mcap_writer",
                    output={"stdout": "screen", "stderr": "screen"},
                    respawn=True,
                    respawn_delay=2.0,
                )
            )

        dc_bridge_node = Node(
            package="dc_bridge",
            executable="dc_bridge",
            name="dc_bridge",
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
            respawn=True,
            respawn_delay=2.0,
            parameters=bridge_parameters,
        )
        extra_actions.extend(build_uploader_action(raw_params))
        return [dc_bridge_node, *extra_actions]

    return OpaqueFunction(function=_build)


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
            # dc_bridge spawns and supervises the Vector shipper and forwards Records to
            # it. It's a plain node outside the lifecycle manager (not in
            # lifecycle_manager_dc's node_names, see ADR-0006), so launch respawn
            # supervises it unconditionally, independent of use_respawn. Also starts
            # dc_mcap_writer alongside it when dc_params_file enables it (ADR-0009,
            # #210) — see build_bridge_and_mcap_actions.
            build_bridge_and_mcap_actions(configured_params),
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
            # dc_bridge isn't built/registered as an rclcpp_components plugin, so it always
            # runs as a plain node (ADR-0006), even when the rest of the stack is composed;
            # launch respawn supervises it unconditionally, independent of use_respawn. Also
            # starts dc_mcap_writer alongside it when dc_params_file enables it (ADR-0009,
            # #210) — see build_bridge_and_mcap_actions.
            build_bridge_and_mcap_actions(configured_params),
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
    ld.add_action(declare_draw_img_service)
    ld.add_action(declare_save_img_service)
    ld.add_action(declare_group_node)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
