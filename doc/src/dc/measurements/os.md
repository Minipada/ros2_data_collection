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
      "description": "Number of CPUs",
      "type": "integer",
      "minimum": 0
    },
    "memory": {
      "description": "System memory",
      "type": "number",
      "minimum": 0
    }
  },
  "type": "object"
}
```

```admonish warning
The Record's own field is `cpus` (plural) — `os.cpp`'s `collect()` writes
`data_json["cpus"]`, not `data_json["cpu"]` as the schema above declares. The
mismatch doesn't fail validation (an extra, unvalidated field isn't rejected without
`additionalProperties: false`), but don't rely on `cpu` showing up in a query.
```

## Measurement configuration

```yaml
...
os:
  plugin: "dc_measurements/OS"
  topic_output: "/dc/measurement/os"
```
