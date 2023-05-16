# Getting started

We will go together through some demos to get started with DC. You shall find them in the *[dc_demos](https://github.com/minipada/ros2_data_collection/tree/humble/dc_demos)* package

```admonish warning
Make sure you [built and sourced](./build.md) the workspace.
```

```admonish info
Have you checked the [configuration examples](./configuration_examples.md) before running the demos? They will help understand how the demo configuration work.
```

| Title                                                                                 | Description                                                                                    |
| ------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| [Uptime](./getting_started/uptime_stdout.md)                                          | Collect how long the system has been running and print it on Stdout. Minimal example           |
| [Group memory and uptime](./getting_started/memory_uptime_stdout.md)                  | Collect both memory and uptime and group them in a dictionary                                  |
| [Turtlebot3 Stdout](./getting_started/tb3_stdout.md)                                  | Collect command velocity, map, position and speed and print it in stdout                       |
| [Turtlebot3 AWS Warehouse MinIO PostgreSQL](./getting_started/tb3_aws_minio_pgsql.md) | Collect system, robot, environment and infrastructure data and send it to MinIO and PostgreSQL |
| [Turtlebot3 QR codes](./getting_started/qrcodes_minio_pgsql.md)                       | Collect QR codes and images                                                                    |
| [Custom plugin](./getting_started/custom_stdout.md)                                   | Create an external plugin                                                                      |

Note that each demo assumes concepts explained in previous getting_started will be acknowledged.
