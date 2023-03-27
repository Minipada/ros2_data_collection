# NULL - Fluent Bit

## Description

The null output plugin just throws away events. See [fluent bit page](https://docs.fluentbit.io/manual/pipeline/outputs/null) for more information.

## Node configuration

```yaml
...
flb_null:
  plugin: "dc_destinations/FlbNull"
  inputs: ["/dc/measurement/data"]
...
```
