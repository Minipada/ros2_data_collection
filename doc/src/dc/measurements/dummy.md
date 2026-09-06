# Dummy

## Description
The dummy measurement, generates dummy events. It is useful for testing, debugging, benchmarking and getting started with ROS 2 Data collection.

## Parameters

| Parameter  | Description        | Type | Default                                 |
| ---------- | ------------------ | ---- | --------------------------------------- |
| **record** | Dummy JSON record. | str  | "{\"message\":\"Hello from ROS 2 DC\"}" |


## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Dummy",
    "description": "Dummy JSON",
    "properties": {
        "message": {
            "description": "Dummy message",
            "type": "string"
        }
    },
    "type": "object"
}
```

## Configuration

```yaml
...
dummy:
  plugin: "dc_measurements/Dummy"
  topic_output: "/dc/measurement/dummy"
```

## Example output

```json
{
  "flattened": false,
  "message": "Hello from ROS 2 DC",
  "name": "dummy",
  "nested": false,
  "run_id": "169"
}
```
