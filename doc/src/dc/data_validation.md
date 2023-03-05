# Data validation

## Model

Each measurement cam validate its data with a model.

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

It is possible to enforce the validation. By default, it is disabled since some fields can be filtered out, and some are added dynamically (e.g remote paths).

Currently it is not possible to select another schema but it is planned to be able to pass a custom path later on.

## Failed validation callback

You might want to trigger some actions when a validation fails, e.g send the data to another database to later on debug it.

In this case, you will need to write your own plugin (inherit from an existing one or start from scratch) and define the `onFailedValidation` function in the class.

This case is covered by the [custom plugin demo](./demos/custom_stdout.md)

## Use a different Schema
For each plugin, a default path is provided but this can be changed by passing the `json_schema_path` parameter in the measurement plugin parameter to the absolute path of your schema.
