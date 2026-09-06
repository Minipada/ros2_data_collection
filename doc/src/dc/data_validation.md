# Data validation

## Model

Each Measurement can validate the Records it emits against a model.

We use use [JSON schema validator for JSON for Modern C++](https://github.com/pboettch/json-schema-validator).

Schemas follow the JSON 7 model:

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Uptime",
    "description": "Time the system has been up",
    "properties": {
        "time": {
            "description": "Time the system has been up",
            "type": "integer",
            "minimum": 0
        }
    },
    "type": "object"
}
```

Validation is **enabled by default** (`enable_validator: true`); disable it per plugin
with `enable_validator: false` when fields are filtered out or added dynamically (e.g.
remote paths) in a way the shipped schema doesn't account for.

## Failed validation callback

You might want to trigger some actions when a validation fails, e.g send the data to another database to later on debug it.

In this case, you will need to write your own plugin (inherit from an existing one or start from scratch) and define the `onFailedValidation` function in the class.

This case is covered by the [custom plugin demo](./demos/custom_stdout.md)

## Use a different Schema
For each plugin, a default path is provided but this can be changed by passing the `json_schema_path` parameter in the measurement plugin parameter to the absolute path of your schema.
