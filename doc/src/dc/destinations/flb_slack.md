# Slack - Fluent Bit

## Description

The Slack output plugin delivers records or messages to your preferred Slack channel. See [fluent bit page](https://docs.fluentbit.io/manual/pipeline/outputs/slack) for more information.

## Parameters

| Parameter   | Description                                        | Type | Default         |
| ----------- | -------------------------------------------------- | ---- | --------------- |
| **webhook** | Absolute address of the Webhook provided by Slack. | str  | N/A (Mandatory) |

## Node configuration

```yaml
...
flb_slack:
  plugin: "dc_destinations/FlbSlack"
  inputs: ["/dc/group/data"]
  webhook: https://hooks.slack.com/services/T00000000/B00000000/XXXXXXXXXXXXXXXXXXXXXXXX
...
```
