# Demos

We will go together through some demos to get started with DC. You shall find them in the *[dc_demos](https://github.com/minipada/ros2_data_collection/tree/humble/dc_demos)* package

```admonish warning
Make sure you [built and sourced](./build.md) the workspace.
```

```admonish info
Have you checked the [configuration examples](./configuration_examples.md) before running the demos? They will help understand how the demo configuration work.
```

| Title                                                                       | Description                                                                                    |
| --------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| [Uptime](./demos/uptime_stdout.md)                                          | Collect how long the system has been running and print it on Stdout. Minimal example           |
| [Group memory and uptime](./demos/memory_uptime_stdout.md)                  | Collect both memory and uptime and group them in a dictionary                                  |
| [Turtlebot3 Stdout](./demos/tb3_stdout.md)                                  | Collect command velocity, map, position and speed and print it in stdout                       |
| [Turtlebot3 AWS Warehouse RustFS PostgreSQL](./demos/tb3_aws_minio_pgsql.md) | Collect system, robot, environment and infrastructure data and send it to RustFS and PostgreSQL |
| [Turtlebot3 AWS Warehouse InfluxDB](./demos/tb3_aws_influxdb.md)            | Collect system, robot, environment and infrastructure data and send it to InfluxDB via the passthrough Destination |
| [Elasticsearch](./demos/elasticsearch.md)                                   | Send system data to Elasticsearch via the passthrough Destination, and look at it in Kibana. No robot or simulator needed |
| [MCAP recording](./demos/mcap_recording.md)                                 | Record system data as .mcap via the passthrough Destination and `dc_mcap_writer`, and open it with `ros2 bag info`/Foxglove. No robot or simulator needed |
| [Turtlebot3 QR codes](./demos/qrcodes_minio_pgsql.md)                       | Collect QR codes and images                                                                    |
| [Custom plugin](./demos/custom_stdout.md)                                   | Create an external plugin                                                                      |

Note that each demo assumes concepts explained in previous demos will be acknowledged.
