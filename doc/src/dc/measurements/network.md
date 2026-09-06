# Network

## Description

Collects ping value, whether or not the PC is online and interfaces available. `ping()`
opens an unprivileged ICMP "ping" socket (`socket(AF_INET, SOCK_DGRAM, IPPROTO_ICMP)`) —
no root or `CAP_NET_RAW` needed, only a permissive `net.ipv4.ping_group_range` (the Linux
default is permissive for the root group, which is what containers and most robot
processes run as). This works unmodified in a plain rootless Podman/Docker container, with
no `--cap-add` or `--privileged` needed.

## Parameters

| Parameter        | Description                                                 | Type | Default   |
| ---------------- | ----------------------------------------------------------- | ---- | --------- |
| **ping_address** | IP address to test the ping to                              | str  | "8.8.8.8" |
| **ping_timeout** | Time in ms before ping times out. Offline if superior to it | int  | 200       |

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Network",
    "description": "Network accessibility and information",
    "properties": {
        "ping": {
            "description": "Time to ping the host in ms",
            "type": "integer",
            "minimum": -1
        },
        "online": {
            "description": "If the pc is online",
            "type": "boolean"
        },
        "interfaces": {
            "description": "List of network interfaces",
            "type": "array",
            "items": {
                "type": "string"
            }
        }
    },
    "type": "object"
}
```

## Configuration

```yaml
...
network:
  plugin: "dc_measurements/Network"
  topic_output: "/dc/measurement/network"
  ping_address: 192.168.0.1
  ping_timeout: 500
```

## Example output

Captured from a plain rootless Podman container, no special flags, `ping_address` pointed
at localhost:

```json
{
  "flattened": false,
  "interfaces": ["lo", "tunl0", "enp0s31f6"],
  "name": "network",
  "nested": false,
  "online": true,
  "ping": 0,
  "run_id": "175"
}
```

The same container against a real external host (`ping_address: 8.8.8.8`):

```json
{
  "flattened": false,
  "interfaces": ["lo", "tunl0", "enp0s31f6"],
  "name": "network",
  "nested": false,
  "online": true,
  "ping": 6,
  "run_id": "176"
}
```

And against an unreachable one (`ping_address: 192.0.2.1`, the timeout dropped to 300ms
for a fast test):

```json
{
  "flattened": false,
  "interfaces": ["lo", "tunl0", "enp0s31f6"],
  "name": "network",
  "nested": false,
  "online": false,
  "ping": -1,
  "run_id": "177"
}
```
