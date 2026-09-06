# Migration

This page tracks in-progress moves between configuration shapes DC itself is making —
distinct from [Configuration examples](./configuration_examples.md), which teaches the
current shape from scratch.

## Blessed `postgres`/`s3`/`console` &rarr; passthrough

[ADR-0003](./adr/0003-blessed-destinations-plus-passthrough.md) blesses `postgres`, `s3`,
`file`, `console` and `vector` with a ROS-param form rendered into Vector config by
`dc_bridge`. An audit of that blessed set found `postgres`, `s3` and `console` carry no
DC-specific logic — they are pure Vector-sink wrappers, and Vector's own `vector validate`
already gives clear, field-level errors for them. `dc_bridge`'s blessed code path for
these three types is being removed
([#472](https://github.com/minipada/ros2_data_collection/issues/472)), once every
in-repo demo, deploy param file and doc using them is moved to passthrough
([#471](https://github.com/minipada/ros2_data_collection/issues/471)). `file` and
`vector` are unaffected — `file` drives the Uploader
([ADR-0005](./adr/0005-file-uploads-are-bridge-responsibility.md)) and `vector` is
reserved for the split-deployment/fleet work in
[#440](https://github.com/minipada/ros2_data_collection/issues/440); neither is a plain
sink wrapper.

**You do not need to wait for #471/#472.** Working passthrough recipes for all three
types — reproducing exactly what `dc_bridge` renders for the blessed form today, verified
against a live Vector instance — are in
[Destinations: Recipes](./destinations.md#recipes-postgres-s3-console-via-passthrough).
Point `custom_config_files` at one of them and drop the equivalent blessed Destination
block; nothing else about your Measurements or routing changes.
