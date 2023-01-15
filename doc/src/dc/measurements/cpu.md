# CPU

## Description

Collect cpu usage: average cpu, number of processes running and processes sorted by cpu usage.

## Parameters

| Parameter         | Description                                                         | Type  | Default |
| ----------------- | ------------------------------------------------------------------- | ----- | ------- |
| **max_processes** | Max amount of processes to collect in the sorted field. -1 for all  | int   | 5       |
| **cpu_min**       | Filters out processes using less than this cpu usage. -1 to disable | float | 5.0     |

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Cpu",
    "description": "CPU statistics",
    "properties": {
        "average": {
            "description": "Average CPU",
            "type": "number",
            "minimum": 0
        },
        "processes": {
            "description": "Number of processes running",
            "type": "integer",
            "minimum": 0
        },
        "sorted": {
            "description": "Processes sorted by CPU usage",
            "type": "array",
            "items": {
                "$ref": "#/$defs/process"
            }
        }
    },
    "$defs": {
        "process": {
            "type": "object",
            "description": "Process information",
            "properties": {
                "pid": {
                    "description": "Process ID of the process",
                    "type": "integer"
                },
                "user": {
                    "description": "User who started the process",
                    "type": "string"
                },
                "cmd": {
                    "description": "Command that launched the process",
                    "type": "string"
                },
                "cpu": {
                    "description": "Process' current utilization as a percentage of total CPU time",
                    "type": "number"
                },
                "ram": {
                    "description": "Memory in use by this process in kb",
                    "type": "integer"
                },
                "uptime": {
                    "description": "Age of the process in seconds",
                    "type": "integer"
                }
            }
        }
    },
    "type": "object"
}
```
