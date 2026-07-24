# DC — ROS 2 Data Collection

DC collects operational data from robots — system, robot-state, and inspection measurements — validates it, and routes it to external infrastructure (databases, object storage, APIs) that powers analytics and dashboards. It is a telemetry pipeline, not an ML dataset platform.

## Language

**Measurement**:
A source of sampled data (CPU, position, camera inspection…) that emits Records.
_Avoid_: metric, sensor log, sample

**Condition**:
A boolean predicate on robot state that gates whether a Measurement's Records are collected.
_Avoid_: trigger, filter

**Group**:
A merge of Records from several Measurements into one Record, based on time proximity.

**Record**:
One timestamped JSON document flowing through the pipeline.
_Avoid_: message, event, log line

**Destination**:
An external system that receives Records or Files (PostgreSQL, S3-compatible storage, console…).
_Avoid_: output, sink (a "sink" is the Shipper's internal configuration unit, not the DC concept)

**Blessed destination**:
A Destination DC configures natively from ROS parameters: PostgreSQL, S3-compatible object storage, file, console.

**Passthrough destination**:
A Destination configured by handing raw Shipper configuration through DC, unlocking the Shipper's full catalog without DC code.

**Bridge**:
The DC component that receives Records from ROS topics and hands them to the Shipper.

**Shipper**:
The external process (Vector by default) that buffers, transforms, and reliably delivers Records to Destinations.
_Avoid_: forwarder, agent, data plane, backend

**Shipper ingest protocol**:
The wire format on the local socket between the Bridge and the Shipper (default port 24224). It is Fluentd's open "Forward" specification — chosen as the cheapest Shipper-native listener with built-in receipt acknowledgement (ADR-0002) — but **no Fluentd or Fluent Bit software runs anywhere in DC 2.0**: the Bridge implements the sender side itself (~170 lines of msgpack). Say "shipper ingest protocol"; the word "fluent" should only appear in the generated Shipper config (`type = "fluent"`) and interop docs.
_Avoid_: "Fluent Forward" as a component name — it is a message format, not software in the pipeline

**File**:
A binary artifact produced by a Measurement (image, video, map) that is uploaded to object storage as-is; only its metadata travels as a Record.

**Tag**:
A label carried by a Record naming a Destination that must receive it.
_Avoid_: route (a "route" is the Shipper-side path a Tag selects)

## Relationships

- A **Measurement** emits **Records**, optionally gated by one or more **Conditions**
- A **Group** merges Records from several **Measurements** into one **Record**
- The **Bridge** forwards every **Record** to the **Shipper**
- The **Shipper** delivers Records to one or more **Destinations**
- A **File** is uploaded to an object-storage **Destination**; its metadata becomes a **Record**
- A **Record** carries **Tags**; each Tag names a **Destination**

## Example dialogue

> **Dev:** "When a camera **Measurement** fires, does the image go through the **Shipper**?"
> **Domain expert:** "No — the image is a **File**; it's uploaded to object storage directly. Only a **Record** with its path, size, and upload status travels through the **Shipper** to PostgreSQL."

## Flagged ambiguities

- "destination" was used for both the DC pluginlib class and the external system — resolved: a **Destination** is the external system; the per-destination pluginlib layer is retired.
- "backend" was used for Fluent Bit's embedded engine — resolved: the engine is the **Shipper**, an external process, not an embedded library.
