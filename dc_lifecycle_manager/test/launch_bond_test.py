import os
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="dc_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_test",
                output="screen",
                parameters=[
                    {"use_sim_time": False},
                    {"autostart": False},
                    {"node_names": ["bond_tester"]},
                ],
            ),
        ]
    )


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    testExecutable = os.getenv("TEST_EXECUTABLE")

    test1_action = ExecuteProcess(cmd=[testExecutable], name="test_bond_gtest", output="screen")

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == "__main__":
    sys.exit(main())
