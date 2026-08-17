# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Send Records to Elasticsearch through the ADR-0003 passthrough Destination.

Hardware-free: four system Measurements (cpu, memory, os, uptime), a `console`
Destination that puts them on the public `dc.<tag>` routes, and a raw Vector
`elasticsearch` sink loaded from `custom_config_files`. See
doc/src/dc/demos/elasticsearch.md.
"""

from dc_demos.launch_common import dc_demo_launch_description


def generate_launch_description():
    return dc_demo_launch_description("elasticsearch.yaml")
