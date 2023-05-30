# InfluxDB - Fluent Bit

## Description

The influxdb output plugin, allows to flush your records into a [InfluxDB](https://www.influxdata.com/time-series-platform/influxdb/) time series database. The following instructions assumes that you have a fully operational InfluxDB service running in your system. See [fluent bit page](https://docs.fluentbit.io/manual/pipeline/outputs/influxdb) for more information.

## Parameters

| Parameter        | Description                                                                                                    | Type        | Default        |
| ---------------- | -------------------------------------------------------------------------------------------------------------- | ----------- | -------------- |
| **host**         | IP address or hostname of the target InfluxDB service.                                                         | str         | "127.0.01"     |
| **port**         | TCP port of the target InfluxDB service.                                                                       | int         | 8086           |
| **database**     | InfluxDB database name where records will be inserted.                                                         | str         | "fluentbit"    |
| **bucket**       | InfluxDB bucket name where records will be inserted - if specified, database is ignored and v2 of API is used. | str         | N/A (Optional) |
| **org**          | InfluxDB organization name where the bucket is (v2 only).                                                      | str         | "fluent"       |
| **sequence_tag** | The name of the tag whose value is incremented for the consecutive simultaneous events.                        | str         | "_seq"         |
| **http_user**    | Optional username for HTTP Basic Authentication.                                                               | str         | N/A (Optional) |
| **http_passwd**  | Password for user defined in HTTP_User.                                                                        | str         | N/A (Optional) |
| **http_token**   | Authentication token used with InfluDB v2 - if specified, both HTTP_User and HTTP_Passwd are ignored.          | str         | N/A (Optional) |
| **tag_keys**     | List of keys that needs to be tagged.                                                                          | list\[str\] | N/A (Optional) |
| **auto_tags**    | Automatically tag keys where value is string.                                                                  | bool        | false          |

## Node configuration

```yaml
...
flb_influxdb:
  plugin: "dc_destinations/FlbInfluxDB"
  inputs: ["/dc/measurement/uptime"]
  host: "127.0.0.1"
  port: 8086
  database: "ros"
...
```
