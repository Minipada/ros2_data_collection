# HTTP - Fluent Bit

## Description

The http output plugin allows to flush your records into a HTTP endpoint. For now the functionality is pretty basic and it issues a POST request with the data records in [MessagePack](http://msgpack.org/) (or JSON) format. See [fluent bit page](https://docs.fluentbit.io/manual/pipeline/outputs/http) for more information.

## Parameters

| Parameter                    | Description                                                                                                                                                                                                                                                                     | Type | Default     |
| ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---- | ----------- |
| **allow_duplicated_headers** | Specify if duplicated headers are allowed. If a duplicated header is found, the latest key/value set is preserved.                                                                                                                                                              | bool | true        |
| **format**                   | Specify the data format to be used in the HTTP request body, by default it uses msgpack. Other supported formats are json, json_stream and json_lines and gelf.                                                                                                                 | str  | "json"      |
| **header**                   | Add a HTTP header key/value pair. Multiple headers can be set.                                                                                                                                                                                                                  | str  | N/A         |
| **headers_key**              | Specify the key to use as the headers of the request (must prefix with "$"). The key must contain a map, which will have the contents merged on the request headers. This can be used for many purposes, such as specifying the content-type of the data contained in body_key. | str  | N/A         |
| **header_tag**               | Specify an optional HTTP header field for the original message tag.                                                                                                                                                                                                             | str  | N/A         |
| **http_user**                | Basic Auth Username.                                                                                                                                                                                                                                                            | str  | N/A         |
| **http_passwd**              | Basic Auth Password. Requires HTTP_User to be set.                                                                                                                                                                                                                              | str  | N/A         |
| **json_date_format**         | Specify the format of the date. Supported formats are double, epoch, iso8601 (eg: 2018-05-30T09:39:52.000681Z) and java_sql_timestamp (eg: 2018-05-30 09:39:52.000681).                                                                                                         | str  | "double"    |
| **json_date_key**            | Specify the name of the time key in the output record. To disable the time key just set the value to false.                                                                                                                                                                     | str  | "date"      |
| **host**                     | IP address or hostname of the target HTTP Server.                                                                                                                                                                                                                               | str  | "127.0.0.1" |
| **log_response_payload**     | Specify if the response paylod should be logged or not.                                                                                                                                                                                                                         | str  | true        |
| **port**                     | TCP port of the target HTTP Server                                                                                                                                                                                                                                              | str  | "80"        |
| **uri**                      | Specify an optional HTTP URI for the target web server, e.g: /something.                                                                                                                                                                                                        | str  | "/"         |
## Node configuration

```yaml
...
flb_http:
  plugin: "dc_destinations/FlbHTTP"
  inputs: ["/dc/measurement/data"]
  host: "127.0.0.1"
  port: 80
  uri: "/"
  format: "json"
...
```
