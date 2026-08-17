# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Print Records from the TurtleBot3 simulation to stdout.

This launch file starts DC only -- Nav2 and the simulator are launched
separately (see doc/src/dc/demos/tb3_stdout.md). It used to also declare slam /
map / world / headless / use_simulator / use_rviz / rviz_config_file /
use_namespace, none of which it ever read: they were advertised by
`--show-args` as if they controlled something, and their defaults pointed at
nav2_bringup/worlds/world_only.model and nav2_bringup/maps/turtlebot3_world.yaml,
both of which Jazzy's nav2_bringup no longer ships (#279).
"""

from dc_demos.launch_common import dc_demo_launch_description


def generate_launch_description():
    return dc_demo_launch_description(
        "tb3_simulation_stdout.yaml",
        group_node="True",
    )
