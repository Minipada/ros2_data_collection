# Infrastructure setup

DC delivers Records and Files to systems you run yourself. These pages cover bringing up
the ones the demos and examples assume:

| Page                                    | Used as                                                        |
| --------------------------------------- | -------------------------------------------------------------- |
| [PostgreSQL](./infrastructure_setup/postgresql.md) | A `postgres` Destination, and the File status log   |
| [RustFS](./infrastructure_setup/rustfs.md)         | An `s3` Destination, for Records and for File uploads |
| [InfluxDB](./infrastructure_setup/influxdb.md)     | A passthrough Destination (`influxdb_logs`)         |
| [IP camera](./infrastructure_setup/ip_camera.md)   | An RTSP source for the IP camera Measurement        |

The compose files live in `tools/infrastructure/docker/`. None of this is required to run
DC — the `console` and `file` Destinations need nothing external.
