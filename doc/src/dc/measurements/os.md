# OS

## Description
Collects the Operating System information: cpus, operating system name and kernel information

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "OS",
    "description": "OS, kernel and CPUs information",
    "properties": {
        "os": {
            "description": "Host distribution name",
            "type": "string"
        },
        "kernel": {
            "description": "Kernel version",
            "type": "string"
        },
        "cpu": {
            "description": "number of CPUs",
            "type": "integer",
            "minimum": 0
        }
    },
    "type": "object"
}
```
