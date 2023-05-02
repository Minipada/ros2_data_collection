# TCP Health

## Description
Collects status of a TCP server.

## Parameters

| Parameter | Description                                     | Type           | Default                |
| --------- | ----------------------------------------------- | -------------- | ---------------------- |
| **name**  | Alias to give to the TCP Server                 | str            | N/A (Mandatory)        |
| **host**  | Name of the target host or IP address to check  | str            | "127.0.0.1" (Optional) |
| **port**  | TCP port where to perform the connection check. | int(>0 <65536) | 80 (Optional)          |

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "TCP Health",
    "description": "Status of a TCP server",
    "properties": {
        "host": {
            "description": "Server hostname",
            "type": "string"
        },
        "port": {
            "description": "Port number",
            "type": "integer",
            "minimum": 1,
            "maximum": 65536
        },
        "server_name": {
            "description": "Time the system has been up",
            "type": "string"
        }
    },
    "type": "object"
}
```

## Measurement configuration

```yaml
...
tcp_health:
  plugin: "dc_measurements/TCPHealth"
  topic_output: "/dc/measurement/minio_health"
  tags: ["flb_stdout"]
  host: "127.0.0.1"
  port: 9000
  name: "minio_api"
```
