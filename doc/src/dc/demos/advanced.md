# Advanced

Either you run the whole inspection pipeline at once — Measurements, Conditions, Groups,
File uploads and the Grafana dashboards reading them back — or you leave YAML behind and
write DC code yourself.

**Prerequisites**: the [Intermediate](./intermediate.md) tier, plus several services running
together for the QR codes pipeline, and a C++ toolchain for the custom plugin. **Roughly 45
minutes** for the QR codes pipeline, **30** for the custom plugin including the rebuild.

| Title                                                    | Description                                                                                             | Also needs                                                                                                                                       |
| ----------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Turtlebot3 QR codes](./qrcodes_minio_pgsql.md)          | Collect QR codes and images, upload them as Files and read them back in Grafana                          | [PostgreSQL](../infrastructure_setup/postgresql.md), [RustFS](../infrastructure_setup/rustfs.md) and Grafana running together, plus the simulator |
| [Fast DDS statistics](./fastdds_stats_pgsql_grafana.md)  | Collect Fast DDS's own network statistics and read them back in Grafana. No robot or simulator needed    | [PostgreSQL](../infrastructure_setup/postgresql.md) and Grafana running together, plus Fast DDS built with its Statistics Module enabled          |
| [Custom plugin](./custom_stdout.md)                      | Create an external plugin                                                                                | No service — a `colcon build` of your own plugin package                                                                                          |

Note that each demo assumes concepts explained in previous demos will be acknowledged.
