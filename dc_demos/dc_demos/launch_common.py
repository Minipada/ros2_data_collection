# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Shared builder for the dc_demos demo launch files.

Every demo in this package brings the same thing up: the standard set of
`dc_bringup` launch arguments, forwarded unchanged to `dc_bringup.launch.py`.
What actually differs between demos is the params file, plus whether the demo
wants the Group node or the image services. Before this module each demo spelled
those ~100 lines of argument declarations out in full, which made eight of the
demo launch files byte-identical apart from a single string.

This is the same pattern as `nav2_common.launch` (which `dc_bringup.launch.py`
already imports): a package's Python module holding launch helpers, imported by
that package's installed launch files. `ament_python_install_package(dc_demos)`
in CMakeLists.txt is what puts this module on the path.

The arguments, their defaults and their descriptions are unchanged from the
hand-written files, so `ros2 launch dc_demos <demo>.launch.py --show-args`
reports what it always did.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def dc_demo_launch_description(
    params_file,
    *,
    use_sim_time="False",
    use_composition="True",
    group_node=None,
    save_img_service=None,
    draw_img_service=None,
    extra_actions=(),
):
    """Build the LaunchDescription for a demo that brings up DC via dc_bringup.

    Args:
        params_file: Basename of the demo's params file in `dc_demos/params`,
            e.g. `"elasticsearch.yaml"`. Becomes the `dc_params_file` default.
        use_sim_time: Default for the `use_sim_time` argument.
        use_composition: Default for the `use_composition` argument.
        group_node: Default for the `group_node` argument, or None to leave it
            undeclared. When None the argument is neither declared nor
            forwarded, so `dc_bringup`'s own default applies — which is what a
            demo that never mentioned `group_node` did before.
        save_img_service: Default for `save_img_service`, or None as above.
        draw_img_service: Default for `draw_img_service`, or None as above.
        extra_actions: Additional actions to add after the bringup include, for
            demos that start something of their own (e.g. a robot state
            publisher).

    Returns:
        The populated LaunchDescription.
    """
    demos_dir = get_package_share_directory("dc_demos")
    bringup_dir = get_package_share_directory("dc_bringup")

    args = [
        ("namespace", "", "Top-level namespace"),
        ("use_sim_time", use_sim_time, "Use simulation (Gazebo) clock if true"),
        (
            "dc_params_file",
            os.path.join(demos_dir, "params", params_file),
            "Full path to the ROS2 parameters file to use for all launched nodes",
        ),
        ("autostart", "True", "Automatically startup the dc stack"),
        ("use_composition", use_composition, "Use composed bringup if True"),
        (
            "container_name",
            "dc_container",
            "the name of container that nodes will load in if use composition",
        ),
        (
            "use_respawn",
            "False",
            "Whether to respawn if a node crashes. Applied when composition is disabled.",
        ),
        ("log_level", "info", "log level"),
    ]

    # Only declared when the demo asks for them; an undeclared argument is also
    # an unforwarded one, leaving dc_bringup's own default in charge.
    args += [
        (name, default, description)
        for name, default, description in (
            ("group_node", group_node, "Start group_node"),
            ("save_img_service", save_img_service, "Start save image service"),
            ("draw_img_service", draw_img_service, "Start draw image service"),
        )
        if default is not None
    ]

    ld = LaunchDescription()

    for name, default, description in args:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "dc_bringup.launch.py")
            ),
            launch_arguments={name: LaunchConfiguration(name) for name, _, _ in args}.items(),
        )
    )

    for action in extra_actions:
        ld.add_action(action)

    return ld
