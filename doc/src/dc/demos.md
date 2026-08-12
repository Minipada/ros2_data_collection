# Demos

We will go together through some demos to get started with DC. You shall find them in the *[dc_demos](https://github.com/minipada/ros2_data_collection/tree/jazzy/dc_demos)* package

```admonish warning
Make sure you [built and sourced](./setup.md#build) the workspace.
```

```admonish info
Have you checked the [configuration examples](./configuration_examples.md) before running the demos? They will help understand how the demo configuration work.
```

The demos are grouped in three tiers. Start at [Beginner](#beginner) and work down: each
demo assumes the concepts explained in the ones before it.

## How the tiers work

A demo's tier is the **heavier of the two things it asks of you**: the infrastructure you
have to run alongside DC, and the DC machinery you have to understand or write. Neither
axis alone is enough — [Custom plugin](./demos/custom_stdout.md) needs no infrastructure
at all but has you writing a C++ Measurement plugin, and
[MCAP recording](./demos/mcap_recording.md) is a single `ros2 launch` away but is built on
the ADR-0003 passthrough.

| Tier                          | Infrastructure to run                                                         | DC machinery involved                                                                              |
| ------------------------------- | ------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| [Beginner](#beginner)         | None. Nothing but the built workspace                                         | Measurements and the blessed `console` Destination, configured in YAML                             |
| [Intermediate](#intermediate) | At most one stack from `tools/infrastructure/docker/`, that you start yourself | The blessed `postgres`/`s3` Destinations, or the ADR-0003 [passthrough](./destinations.md)         |
| [Advanced](#advanced)         | The full inspection stack — PostgreSQL, RustFS and Grafana at once             | Code you write yourself, or Measurements, Conditions, Groups, Files and dashboards wired end to end |

When adding a demo, find the heaviest thing it asks of the reader — a service to stand up,
or code to write — and file it under the matching tier.

## Beginner

Everything prints to your terminal: nothing to install, start or clean up afterwards, and
nothing to go and look at in another tool.

**Prerequisites**: a [built and sourced](./setup.md#build) workspace, and nothing else — no
containers, no databases. **Roughly 5 minutes each**, 15 for the Turtlebot3 one including
simulator startup.

| Title                                                      | Description                                                                          | Also needs                                          |
| ------------------------------------------------------------ | -------------------------------------------------------------------------------------- | ----------------------------------------------------- |
| [Uptime](./demos/uptime_stdout.md)                         | Collect how long the system has been running and print it on Stdout. Minimal example | —                                                   |
| [Group memory and uptime](./demos/memory_uptime_stdout.md) | Collect both memory and uptime and group them in a dictionary                        | —                                                   |
| [Turtlebot3 Stdout](./demos/tb3_stdout.md)                 | Collect command velocity, map, position and speed and print it in stdout             | The Nav2 Turtlebot3 simulation (see [Setup](./setup.md)) |

## Intermediate

The Records leave the terminal: they land in a database, a search index, a bucket or a
file, and you go and read them back there. This is the tier where Destinations — blessed
and passthrough — are introduced.

**Prerequisites**: the Beginner tier, plus the one service each demo sends to, started from
`tools/infrastructure/docker/` as described on its [Infrastructure
setup](./infrastructure_setup.md) page. The two Turtlebot3 demos also need the AWS
warehouse world on top of the simulator. **Roughly 20 to 30 minutes each**, plus a one-off
container image pull the first time you bring a stack up.

| Title                                                                       | Description                                                                                                                                              | Also needs                                                                                                                  |
| ----------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------- |
| [MCAP recording](./demos/mcap_recording.md)                                 | Record system data as .mcap via the passthrough Destination and `dc_mcap_writer`, and open it with `ros2 bag info`/Foxglove. No robot or simulator needed | No service — just an MCAP viewer (`ros2 bag info`, [Foxglove](https://foxglove.dev/)) to read the result                     |
| [Elasticsearch](./demos/elasticsearch.md)                                   | Send system data to Elasticsearch via the passthrough Destination, and look at it in Kibana. No robot or simulator needed                                 | [Elasticsearch + Kibana](./infrastructure_setup/elasticsearch.md)                                                            |
| [Turtlebot3 AWS Warehouse RustFS PostgreSQL](./demos/tb3_aws_minio_pgsql.md) | Collect system, robot, environment and infrastructure data and send it to RustFS and PostgreSQL                                                           | [PostgreSQL](./infrastructure_setup/postgresql.md) + [RustFS](./infrastructure_setup/rustfs.md), and the AWS warehouse world |
| [Turtlebot3 AWS Warehouse InfluxDB](./demos/tb3_aws_influxdb.md)            | Collect system, robot, environment and infrastructure data and send it to InfluxDB via the passthrough Destination                                        | [InfluxDB](./infrastructure_setup/influxdb.md), and the AWS warehouse world                                                 |

## Advanced

Either you run the whole inspection pipeline at once — Measurements, Conditions, Groups,
File uploads and the Grafana dashboards reading them back — or you leave YAML behind and
write DC code yourself.

**Prerequisites**: the Intermediate tier, plus several services running together for the QR
codes pipeline, and a C++ toolchain for the custom plugin. **Roughly 45 minutes** for the
QR codes pipeline, **30** for the custom plugin including the rebuild.

| Title                                                 | Description                                                                     | Also needs                                                                                                                                     |
| ------------------------------------------------------- | --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| [Turtlebot3 QR codes](./demos/qrcodes_minio_pgsql.md) | Collect QR codes and images, upload them as Files and read them back in Grafana | [PostgreSQL](./infrastructure_setup/postgresql.md), [RustFS](./infrastructure_setup/rustfs.md) and Grafana running together, plus the simulator |
| [Custom plugin](./demos/custom_stdout.md)             | Create an external plugin                                                       | No service — a `colcon build` of your own plugin package                                                                                       |

Note that each demo assumes concepts explained in previous demos will be acknowledged.
