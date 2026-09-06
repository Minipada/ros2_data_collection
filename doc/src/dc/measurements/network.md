# Network

## Description

Collects ping value, whether or not the PC is online and interfaces available.

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

Not yet captured: `ping()` opens a raw ICMP socket (`socket(AF_INET, SOCK_RAW,
IPPROTO_ICMP)`), which rootless Podman refuses outright —
`[ICMP] unknown protocol or Permission denied, try again with root permissions`, logged at
configure time — regardless of `--cap-add=NET_RAW` or even `--privileged`: a rootless
container's process is still an unprivileged UID from the kernel's point of view, and raw
sockets aren't governed by the namespaced-capability path that `--cap-add`/`--privileged`
actually grant there. `interfaces` still populates correctly (that part doesn't need raw
sockets); only `ping`/`online` need capturing on a real robot or genuinely rootful Docker/
Podman instead.
