# TCP & TLS - Fluent Bit

## Description

The tcp output plugin allows to send records to a remote TCP server. The payload can be formatted in different ways as required.. See [fluent bit page](https://docs.fluentbit.io/manual/pipeline/outputs/tcp-and-tls) for more information.

## Parameters

| Parameter            | Description                                                                                                                                                            | Type           | Default        |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------- | -------------- |
| **host**             | Target host where Fluent-Bit or Fluentd are listening for Forward messages.                                                                                            | str            | "127.0.0.1"    |
| **port**             | TCP Port of the target service.                                                                                                                                        | int(>0 <65536) | 5170           |
| **format**           | Specify the data format to be printed. Supported formats are msgpack json, json_lines and json_stream.                                                                 | str            | "msgpack"      |
| **json_date_key**    | Specify the name of the time key in the output record. To disable the time key just set the value to false.                                                            | str            | "date"         |
| **json_date_format** | Specify the format of the date. Supported formats are double, epoch, iso8601 (eg: 2018-05-30T09:39:52.000681Z) and java_sql_timestamp (eg: 2018-05-30 09:39:52.000681) | str            | "double"       |
| **workers**          | Enables dedicated thread(s) for this output. Default value is set since version 1.8.13. For previous versions is 0.                                                    | int            | 2              |
| **tls.active**       | Enable or disable TLS support.                                                                                                                                         | bool           | false          |
| **tls.verify**       | Force certificate validation.                                                                                                                                          | bool           | true           |
| **tls.debug**        | Set TLS debug verbosity level. It accept the following values: 0 (No debug), 1 (Error), 2 (State change), 3 (Informational) and 4 Verbose.                             | int(>=0 <=4)   | 1              |
| **tls.ca_file**      | Absolute path to CA certificate file.                                                                                                                                  | str            | N/A (Optional) |
| **tls.crt_file**     | Absolute path to Certificate file.                                                                                                                                     | str            | N/A (Optional) |
| **tls.key_file**     | Absolute path to private Key file.                                                                                                                                     | str            | N/A (Optional) |
| **tls.key_passwd**   | Optional password for tls.key_file file.                                                                                                                               | str            | N/A (Optional) |

## Node configuration

```yaml
...
flb_tcp:
  plugin: "dc_destinations/FlbTCP"
  inputs: ["/dc/measurement/uptime"]
  host: "127.0.0.1"
  port: 5170
...
```
