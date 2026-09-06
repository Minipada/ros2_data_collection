# Intermediate

The Records leave the terminal: they land in a database, a search index, a bucket or a
file, and you go and read them back there. This is the tier where Destinations — blessed
and passthrough — are introduced.

**Prerequisites**: the [Beginner](./beginner.md) tier, plus the one service each demo sends
to, started from `tools/infrastructure/docker/` as described on its [Infrastructure
setup](../infrastructure_setup.md) page. The two Turtlebot3 demos also need the AWS
warehouse world on top of the simulator. **Roughly 20 to 30 minutes each**, plus a one-off
container image pull the first time you bring a stack up.

| Title                                                                  | Description                                                                                                                                                | Also needs                                                                                                                    |
| ------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| [MCAP recording](./mcap_recording.md)                                  | Record system data as .mcap via the passthrough Destination and `dc_mcap_writer`, and open it with `ros2 bag info`/Foxglove. No robot or simulator needed | No service — just an MCAP viewer (`ros2 bag info`, [Foxglove](https://foxglove.dev/)) to read the result                        |
| [Elasticsearch](./elasticsearch.md)                                     | Send system data to Elasticsearch via the passthrough Destination, and look at it in Kibana. No robot or simulator needed                                 | [Elasticsearch + Kibana](../infrastructure_setup/elasticsearch.md)                                                                |
| [Turtlebot3 AWS Warehouse RustFS PostgreSQL](./tb3_aws_minio_pgsql.md) | Collect system, robot, environment and infrastructure data and send it to RustFS and PostgreSQL                                                            | [PostgreSQL](../infrastructure_setup/postgresql.md) + [RustFS](../infrastructure_setup/rustfs.md), and the AWS warehouse world |
| [Turtlebot3 AWS Warehouse InfluxDB](./tb3_aws_influxdb.md)             | Collect system, robot, environment and infrastructure data and send it to InfluxDB via the passthrough Destination                                        | [InfluxDB](../infrastructure_setup/influxdb.md), and the AWS warehouse world                                                    |
