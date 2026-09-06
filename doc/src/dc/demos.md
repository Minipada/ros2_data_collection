# Demos

We will go together through some demos to get started with DC. You shall find them in the *[dc_demos](https://github.com/minipada/ros2_data_collection/tree/jazzy/dc_demos)* package

The demos are grouped in three tiers. Start at [Beginner](./demos/beginner.md) and work
down: each demo assumes the concepts explained in the ones before it.

## How the tiers work

A demo's tier is the **heavier of the two things it asks of you**: the infrastructure you
have to run alongside DC, and the DC machinery you have to understand or write. Neither
axis alone is enough — [Custom plugin](./demos/custom_stdout.md) needs no infrastructure
at all but has you writing a C++ Measurement plugin, and
[MCAP recording](./demos/mcap_recording.md) is a single `ros2 launch` away but is built on
the ADR-0003 passthrough.

| Tier                                    | Infrastructure to run                                                          | DC machinery involved                                                                               |
| ------------------------------------------ | ---------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| [Beginner](./demos/beginner.md)         | None. Nothing but the built workspace                                          | Measurements and the blessed `console` Destination, configured in YAML                              |
| [Intermediate](./demos/intermediate.md) | At most one stack from `tools/infrastructure/docker/`, that you start yourself | The blessed `postgres`/`s3` Destinations, or the ADR-0003 [passthrough](./destinations.md)          |
| [Advanced](./demos/advanced.md)         | The full inspection stack — PostgreSQL, RustFS and Grafana at once             | Code you write yourself, or Measurements, Conditions, Groups, Files and dashboards wired end to end |

When adding a demo, find the heaviest thing it asks of the reader — a service to stand up,
or code to write — and file it under the matching tier.
