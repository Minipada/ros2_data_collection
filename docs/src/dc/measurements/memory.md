# Memory

## Description

Collect memory used in percentage.

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Memory",
    "description": "Memory used",
    "properties": {
        "used": {
            "description": "Memory used in percent",
            "type": "number",
            "minimum": 0
        }
    },
    "type": "object"
}
```

## Measurement configuration

```yaml
...
memory:
  plugin: "dc_measurements/Memory"
  topic_output: "/dc/measurement/memory"
  tags: ["flb_stdout"]
```
