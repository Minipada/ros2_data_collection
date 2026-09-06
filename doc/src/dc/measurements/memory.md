# Memory

## Description

Collect memory used in percentage.

## Parameters

This Measurement has no parameters beyond the [common Plugin parameters](../measurements.md#plugin-parameters).

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

## Configuration

```yaml
...
memory:
  plugin: "dc_measurements/Memory"
  topic_output: "/dc/measurement/memory"
  tags: ["console"]
```

## Example output

```json
{
  "flattened": false,
  "name": "memory",
  "nested": false,
  "run_id": "169",
  "used": 91.44149017333984
}
```
