"""Send QR-code inspection Records to PostgreSQL and object storage.

Starts DC plus a robot state publisher for the camera-equipped TurtleBot3 URDF --
Nav2 and the simulator are launched separately. See
doc/src/dc/demos/qrcodes_minio_pgsql.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from dc_demos.launch_common import dc_demo_launch_description


def generate_launch_description():
    dc_description_dir = get_package_share_directory("dc_description")

    # Use custom URDF with camera
    urdf = os.path.join(dc_description_dir, "urdf", "turtlebot3_waffle_qrcodes.xacro")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        arguments=["urdf", urdf],
    )

    return dc_demo_launch_description(
        "qrcodes_minio_pgsql.yaml",
        use_composition="False",
        group_node="True",
        save_img_service="True",
        draw_img_service="True",
        extra_actions=(start_robot_state_publisher_cmd,),
    )
