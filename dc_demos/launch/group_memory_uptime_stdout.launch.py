"""Merge memory and uptime Records with the Group node, then print to stdout.

See doc/src/dc/demos/memory_uptime_stdout.md.
"""

from dc_demos.launch_common import dc_demo_launch_description


def generate_launch_description():
    return dc_demo_launch_description(
        "group_memory_uptime_stdout.yaml",
        group_node="True",
        # Lowercase "false" here, "False" in every other demo. Launch coerces
        # both to the same boolean, so this is only an inconsistency -- kept
        # verbatim so this refactor stays a provable no-op.
        use_sim_time="false",
    )
