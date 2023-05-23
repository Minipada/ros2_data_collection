# AWS Kinesis Data Streams - Fluent Bit

## Description

The Amazon Kinesis Data Streams output plugin allows to ingest your records into the [Kinesis](https://aws.amazon.com/kinesis/data-streams/) service. See [fluent bit page](https://docs.fluentbit.io/manual/pipeline/outputs/kinesis) for more information.

## Parameters

| Parameter               | Description                                                                                                                                                                                                                                                                                                                            | Type | Default             |
| ----------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---- | ------------------- |
| **region**              | The AWS region.                                                                                                                                                                                                                                                                                                                        | str  | N/A                 |
| **stream**              | The name of the Kinesis Streams Delivery stream that you want log records sent to.                                                                                                                                                                                                                                                     | str  | N/A                 |
| **role_arn**            | ARN of an IAM role to assume (for cross account access).                                                                                                                                                                                                                                                                               | str  | N/A                 |
| **time_key**            | Add the timestamp to the record under this key. By default the timestamp from Fluent Bit will not be added to records sent to Kinesis.                                                                                                                                                                                                 | str  | N/A                 |
| **time_key_format**     | strftime compliant format string for the timestamp; for example, the default is '%Y-%m-%dT%H:%M:%S'. Supports millisecond precision with '%3N' and supports nanosecond precision with '%9N' and '%L'; for example, adding '%3N' to support millisecond '%Y-%m-%dT%H:%M:%S.%3N'. This option is used with time_key.                     | str  | "%Y-%m-%dT%H:%M:%S" |
| **log_key**             | By default, the whole log record will be sent to Kinesis. If you specify a key name with this option, then only the value of that key will be sent to Kinesis. For example, if you are using the Fluentd Docker log driver, you can specify log_key log and only the log message will be sent to Kinesis.                              | str  | N/A                 |
| **endpoint**            | Specify a custom endpoint for the Kinesis API.                                                                                                                                                                                                                                                                                         | str  | N/A                 |
| **auto_retry_requests** | Immediately retry failed requests to AWS services once. This option does not affect the normal Fluent Bit retry mechanism with backoff. Instead, it enables an immediate retry with no delay for networking errors, which may help improve throughput when there are transient/random networking issues. This option defaults to true. | bool | true                |

## Node configuration

```yaml
...
flb_kinesis_streams:
  plugin: "dc_destinations/FlbKinesisStreams"
  inputs: ["/dc/measurement/data"]
  region: "us-east-1"
  stream: my_stream
  time_key: "date"
  time_key_format: "%Y-%m-%dT%H:%M:%S"
...
```
