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
            "description": "Server alias, from the 'name' parameter",
            "type": "string"
        },
        "active": {
            "description": "Whether the TCP connection check succeeded",
            "type": "boolean"
        }
    },
    "type": "object"
}
```

```admonish warning
`tcp_health.json` (the schema file this block is copied from) is wrong twice: `active`
— the boolean connection-health result `collect()` actually writes — isn't declared as
a property at all, and `server_name`'s own description says "Time the system has been
up", copy-pasted from `uptime.json`. Corrected above; the schema file itself still has
both bugs.
```

## Configuration

```yaml
...
tcp_health:
  plugin: "dc_measurements/TCPHealth"
  topic_output: "/dc/measurement/rustfs_health"
  group_key: "rustfs_health"
  host: "127.0.0.1"
  port: 9000
  name: "rustfs_api"
```

## Example output

```json
{
  "active": true,
  "flattened": false,
  "host": "127.0.0.1",
  "name": "tcp_health",
  "nested": false,
  "port": 9000,
  "run_id": "169",
  "server_name": "test_service"
}
```
