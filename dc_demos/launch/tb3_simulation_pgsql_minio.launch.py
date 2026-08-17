# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Send Records from the TurtleBot3 simulation to PostgreSQL and object storage.

This launch file starts DC only -- Nav2 and the simulator are launched
separately. See doc/src/dc/demos/tb3_aws_minio_pgsql.md.
"""

from dc_demos.launch_common import dc_demo_launch_description


def generate_launch_description():
    return dc_demo_launch_description(
        "tb3_simulation_pgsql_minio.yaml",
        group_node="False",
    )
