# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Raw-topic bringup (#227): `dc_bridge` alone, collecting the ROS graph itself.

The DC 2.0 equivalent of the original "only start the destination server" idea. Nothing
here declares a Measurement, so none of the collection stack is launched: no
`measurement_server`, no `group_server`, no `lifecycle_manager_dc`, and no readiness
gate — the gate exists to hold *collection nodes* back until the Bridge can accept their
Records (ADR-0006), and with no collection nodes there is nothing to hold back. The
Bridge subscribes to whatever the `raw:` block in the params file matches, whenever it
appears on the graph.

    ros2 launch dc_bringup dc_raw.launch.py

Point it at your own file with `dc_params_file:=/path/to/params.yaml`; the default
(`params/dc_raw_params.yaml`) dumps everything but the ROS infrastructure topics to
stdout. See doc/src/dc/raw_topics.md.

This is a *complement* to the normal bringup, not a replacement: raw mode ships whatever
a topic happens to contain, with no validation, no Conditions, no Groups and no File
uploads. Use `dc_bringup.launch.py` (which can also enable `raw:` alongside real
Measurements) when you want any of that.
"""

import os
import shutil

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _stage_custom_config_files(params_file_path, custom_config_files):
    """Same helper as dc_bringup.launch.py's — see that module for the full docstring.

    Duplicated rather than imported: launch files are executed as standalone scripts
    by `ros2 launch`, not reliably importable as a package module. Stages a demo's
    passthrough sink TOML(s) at the path `custom_config_files` names, from this
    package's `config/` directory (installed as a sibling of `params/` by
    dc_bringup/CMakeLists.txt), so `ros2 launch dc_bringup dc_raw.launch.py` keeps
    working with zero manual setup now that `console` is passthrough-only (#472).
    """
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(params_file_path))), "config"
    )
    for raw_path in custom_config_files:
        dest_path = os.path.expanduser(os.path.expandvars(raw_path))
        if os.path.exists(dest_path):
            continue
        candidate = os.path.join(config_dir, os.path.basename(dest_path))
        if os.path.isfile(candidate):
            os.makedirs(os.path.dirname(dest_path), exist_ok=True)
            shutil.copyfile(candidate, dest_path)


def generate_launch_description():
    bringup_dir = get_package_share_directory("dc_bringup")

    namespace = LaunchConfiguration("namespace")
    dc_params_file = LaunchConfiguration("dc_params_file")
    log_level = LaunchConfiguration("log_level")

    configured_params = RewrittenYaml(
        source_file=dc_params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )

    def _stage(context, *args, **kwargs):
        params_file_path = LaunchConfiguration("dc_params_file").perform(context)
        try:
            with open(params_file_path) as f:
                raw_params = yaml.safe_load(f) or {}
        except OSError:
            raw_params = {}
        dc_bridge_params = (raw_params.get("dc_bridge") or {}).get("ros__parameters") or {}
        _stage_custom_config_files(
            params_file_path, dc_bridge_params.get("custom_config_files", [])
        )
        return []

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace"),
            DeclareLaunchArgument(
                "dc_params_file",
                default_value=os.path.join(bringup_dir, "params", "dc_raw_params.yaml"),
                description="Full path to the ROS 2 parameters file to use for dc_bridge",
            ),
            DeclareLaunchArgument("log_level", default_value="info", description="log level"),
            OpaqueFunction(function=_stage),
            # Same node, same respawn policy as in dc_bringup.launch.py: dc_bridge
            # spawns and supervises the Vector shipper, and stays outside the lifecycle
            # manager (ADR-0006).
            Node(
                package="dc_bridge",
                executable="dc_bridge",
                name="dc_bridge",
                output={"stdout": "screen", "stderr": "screen"},
                respawn=True,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
