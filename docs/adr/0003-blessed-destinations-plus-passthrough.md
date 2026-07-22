# Destination config: blessed set via ROS params, everything else via passthrough

The pluginlib destination-plugin layer is retired. DC generates shipper sink config from plain ROS parameters for a blessed set only — PostgreSQL, S3-compatible object storage, file, console — via templating in the Bridge. Every other destination (Kinesis, InfluxDB, Slack, Kafka, …) is configured by passing raw Vector sink config through a `custom_sinks` parameter, which exposes the shipper's entire catalog at zero DC code and keeps the "one tool handles all destinations" promise.

## Consequences

- Every Destination — whether it receives Records or Files — is declared in the single `destinations` list, distinguished by a `receives: records|files` property; only File *policy* (delete-after-verified-upload, metadata destination) lives in a separate block. This keeps the config aligned with the glossary: a Destination is a Destination.
- The passthrough routing contract is public API: the Bridge exposes one Shipper route per Tag under the stable name `dc.<tag>`, and custom sink snippets consume those names. This name is chosen deliberately here so implementation doesn't improvise it.

## Considered Options

- Keep per-destination pluginlib plugins (rejected: permanent per-sink maintenance; the layer was only ever plumbing around Fluent Bit's C API)
- Raw Vector config only (rejected: loses ROS-native UX for the 90% path)
