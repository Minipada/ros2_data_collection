# Demos

We will go together through some demos to get started with DC. You shall find them in the *[dc_demos](https://github.com/minipada/ros2_data_collection/tree/humble/dc_demos)* package

```admonish warning
Make sure you [built and sourced](./build.md) the workspace.
```

```admonish info
Have you checked the [configuration examples](./configuration_examples.md) before running the demos? They will help understand how the demo configuration work.
```

| Title                                                      | Description                                                                         |
| ---------------------------------------------------------- | ----------------------------------------------------------------------------------- |
| [Uptime](./demos/uptime_stdout.md)                         | Minimal example of configuration, collects how long your system has been running    |
| [Group memory and uptime](./demos/memory_uptime_stdout.md) | In this demo, we will collect both memory and uptime and group them in a dictionary |
| [Turtlebot3 QR codes](./demos/tb3_stdout_minio_pgsql.md)   | Collect QR codes and images                                                         |
| [Custom plugin](./demos/custom_stdout.md)                  | Create an external plugin                                                           |

Note that each demo assumes concepts explained in previous demos will be acknowledged.
