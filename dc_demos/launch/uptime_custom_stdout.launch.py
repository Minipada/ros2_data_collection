"""Print Records from the custom uptime Measurement plugin to stdout.

The plugin itself lives in dc_demos/plugins/measurements/uptime_custom.cpp.
See doc/src/dc/demos/custom_stdout.md.
"""

from dc_demos.launch_common import dc_demo_launch_description


def generate_launch_description():
    return dc_demo_launch_description("uptime_custom_stdout.yaml")
