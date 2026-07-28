# Random

## Description
Emits a randomly generated value on every polling interval. Useful for exercising the
pipeline and Destinations without any robot infrastructure — the simplest possible
Measurement plugin, and a deterministic, infrastructure-free Record source for load and
backpressure testing.

## Parameters

| Parameter | Description                                                                                                          | Type   | Default               |
| --------- | --------------------------------------------------------------------------------------------------------------------- | ------ | ---------------------- |
| **type**  | Type of the generated value: `integer` or `double`                                                                    | str    | "integer" (Optional)   |
| **min**   | Minimum value (inclusive) of the generated range                                                                       | double | 0.0 (Optional)         |
| **max**   | Maximum value of the generated range (inclusive for `integer`, effectively exclusive for `double`). Must be greater than **min** | double | 100.0 (Optional) |
| **seed**  | Seed for the random number generator. A negative value seeds from a non-deterministic source; a non-negative value makes runs reproducible | int    | -1 (Optional)           |

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Random",
    "description": "Randomly generated value",
    "properties": {
        "value": {
            "description": "Randomly generated value within the configured range",
            "type": "number"
        }
    },
    "type": "object"
}
```

## Measurement configuration

```yaml
...
random:
  plugin: "dc_measurements/Random"
  topic_output: "/dc/measurement/random"
  tags: ["console"]
  type: "double"
  min: 0.0
  max: 1.0
  seed: 42
```
