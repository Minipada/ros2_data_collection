# Migration

This page tracks in-progress moves between configuration shapes DC itself is making —
distinct from [Configuration examples](./configuration_examples.md), which teaches the
current shape from scratch.

## Blessed `postgres`/`s3`/`console` &rarr; passthrough (completed)

[ADR-0003](./adr/0003-blessed-destinations-plus-passthrough.md) originally blessed
`postgres`, `s3`, `file`, `console` and `vector` with a ROS-param form rendered into
Vector config by `dc_bridge`. An audit of that blessed set found `postgres`, `s3` (for
`receives: records`) and `console` carried no DC-specific logic — they were pure
Vector-sink wrappers, and Vector's own `vector validate` already gives clear,
field-level errors for them. Every in-repo demo, deploy param file and doc using them
was moved to passthrough first
([#471](https://github.com/minipada/ros2_data_collection/issues/471)), then
`dc_bridge`'s blessed code path for these three types was removed
([#472](https://github.com/minipada/ros2_data_collection/issues/472)) — as of that
change, `postgres`, `s3` (for `receives: records`) and `console` are **only**
configurable via passthrough; the blessed ROS-param form for them no longer exists.

`file` and `vector` are unaffected and remain blessed — `file` is a plain local-disk
Vector sink with no DC-specific logic either, but is kept as the cheap "anchor"
Destination every passthrough setup needs (`dc_bridge` derives its ROS subscriptions and
`dc.<tag>` routes from `destinations` alone, never from a passthrough snippet's
`inputs`); `vector` is reserved for the split-deployment/fleet work in
[#440](https://github.com/minipada/ros2_data_collection/issues/440). `s3` also stays
blessed for `receives: files` (File uploads,
[ADR-0005](./adr/0005-file-uploads-are-bridge-responsibility.md)) — that path is served
entirely by the Uploader's own S3 client, never by a Vector sink, so there is no
passthrough equivalent for it to migrate to.

Working passthrough recipes for `postgres`, `s3` (records) and `console` — reproducing
exactly what `dc_bridge` used to render for the blessed form — are in
[Destinations: Recipes](./destinations.md#recipes-postgres-s3-console-via-passthrough).
Point `custom_config_files` at one of them, alongside a `file` (or `vector`) anchor
Destination; nothing else about your Measurements or routing changes.
