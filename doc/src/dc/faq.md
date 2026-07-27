# FAQ

## I can't find a Measurement I need

Measurements will keep being added, but the current focus is on getting feedback, fixing
bugs, documentation and reaching a minimum test coverage.

Create a feature request in [Github Discussions](https://github.com/Minipada/ros2_data_collection/discussions/categories/ideas-and-feature-requests)...or better, write your plugin and open a Pull Request.

## I can't find the Destination I need

You do not need one to exist. The four **blessed** Destination types are the ones DC
configures natively from ROS parameters; everything else in Vector's catalog works today
through the [passthrough](./destinations.md#passthrough-custom_config_files) — see the
question below.

## I'm coming from DC 1.x and my `flb_*` destinations are gone

They were replaced, not dropped. The [migration guide](./migration.md) maps every
DC 1.x Destination to its DC 2.0 equivalent with before/after configuration.

## How can I send data to a Destination that isn't blessed?

The Bridge (`dc_bridge`) renders its Shipper's (Vector) config from plain ROS
parameters for a **blessed set** of Destination types only (PostgreSQL,
S3-compatible storage, file, console — see [Destinations](./destinations.md)). Every
other sink in [Vector's catalog](https://vector.dev/docs/reference/configuration/sinks/)
(Kafka, Kinesis, InfluxDB, webhooks, …) is reachable through the **passthrough**: list a
raw Vector config snippet (TOML) in the `custom_config_files` parameter, and consume the
public `dc.<tag>` route it needs. No DC code, plugin, or extra language required — only
Vector configuration.

## Can a passthrough snippet be generated or written in a language other than TOML?

The snippet the Bridge merges in must be Vector's own TOML configuration syntax — DC
does not transform it. If you would rather generate that TOML from another language or
tool, nothing stops you from doing so as a build or deploy step; the Bridge only reads
the resulting file.

## My group data is not published on the group topic

This may happen for different reasons:

1. The group node is not started, be sure it is (ros2 node list), you will need to enable it in your launch file or using the `group_node:=true` when launching the bringup
2. Data is not being published on all topics it subscribes to (use ros2 topic echo on each to ensure that). Currently, if all topics don't publish, there is no way to ignore that and still send the message partially full (see [here](https://answers.ros.org/question/410138/is-it-possible-to-drop-or-keep-message-in-approximatetimesynchronizer/)). Help is welcome here :)
