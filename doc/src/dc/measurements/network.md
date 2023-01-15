# Network

## Description

Collects ping value, whether or not the PC is online and interfaces available.

## Parameters

| Parameter        | Description                                                 | Type | Default   |
| ---------------- | ----------------------------------------------------------- | ---- | --------- |
| **ping_address** | IP address to test the ping to                              | str  | "8.8.8.8" |
| **ping_timeout** | Time in ms before ping times out. Offline if superior to it | int  | 5000      |

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
