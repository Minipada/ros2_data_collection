# InfluxDB

## Description
[InfluxDB](https://www.influxdata.com/) is a time-series database purpose-built for
metrics and events. DC has no blessed `influxdb` Destination — Vector's `influxdb_logs`
sink is reached through the [passthrough Destination](../destinations.md#passthrough-custom_config_files),
the same mechanism the [InfluxDB demo](../demos/tb3_aws_influxdb.md) uses.

## Start in a container
Execute:

```bash
./tools/infrastructure/scripts/install_infrastructure.bash \
  --tool=influxdb \
  --install-type=docker
```

## Start natively

```bash
./tools/infrastructure/scripts/install_infrastructure.bash \
  --tool=influxdb \
  --install-type=native
```

## Credentials

| User  | Password | Database | Port |
| ----- | -------- | -------- | ---- |
| admin | admin    | dc       | 8086 |
