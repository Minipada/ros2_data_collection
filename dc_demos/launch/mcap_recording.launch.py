"""Record Records as .mcap through the ADR-0003/ADR-0009 passthrough (#210).

Hardware-free: four system Measurements (cpu, memory, os, uptime), a `console`
Destination that puts them on the public `dc.<tag>` routes, and a raw Vector `socket`
sink loaded from `custom_config_files` that streams them to the standalone
`dc_mcap_writer` process. See doc/src/dc/demos/mcap_recording.md.
"""

from dc_demos.launch_common import dc_demo_launch_description


def generate_launch_description():
    return dc_demo_launch_description("mcap_recording.yaml")
