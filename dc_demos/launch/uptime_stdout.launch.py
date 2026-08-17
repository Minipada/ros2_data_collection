# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Print uptime Records to stdout — the smallest hardware-free demo.

See doc/src/dc/demos/uptime_stdout.md.
"""

from dc_demos.launch_common import dc_demo_launch_description


def generate_launch_description():
    return dc_demo_launch_description("uptime_stdout.yaml")
