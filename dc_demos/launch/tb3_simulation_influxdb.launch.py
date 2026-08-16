"""Send Records from the TurtleBot3 simulation to InfluxDB.

Uses the ADR-0003 passthrough Destination with a raw Vector `influxdb_metrics`
sink from `custom_config_files`. This launch file starts DC only -- Nav2 and the
simulator are launched separately. See doc/src/dc/demos/tb3_aws_influxdb.md.
"""

from dc_demos.launch_common import dc_demo_launch_description


def generate_launch_description():
    return dc_demo_launch_description(
        "tb3_simulation_influxdb.yaml",
        group_node="False",
    )
