# File - Fluent Bit

## Description

The stdout output plugin allows to print to the standard output the data received through the input plugin. See [fluent bit page](https://docs.fluentbit.io/manual/pipeline/outputs/standard-output) for more information.

## Parameters

| Parameter            | Description                                                                                                                                                             | Type | Default  |
| -------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---- | -------- |
| **format**           | Specify the data format to be printed. Supported formats are msgpack json, json_lines and json_stream.                                                                  | str  | "json"   |
| **json_date_key**    | Specify the name of the time key in the output record. To disable the time key just set the value to false.                                                             | str  | "date"   |
| **json_date_format** | Specify the format of the date. Supported formats are double, epoch, iso8601 (eg: 2018-05-30T09:39:52.000681Z) and java_sql_timestamp (eg: 2018-05-30 09:39:52.000681). | str  | "double" |

## Node configuration

```yaml
...
flb_stdout:
  plugin: "dc_destinations/FlbStdout"
  inputs: ["/dc/group/data"]
  format: "json"
  json_date_key: "date"
  json_date_format: "iso8601"
...
```
