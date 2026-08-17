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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


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
