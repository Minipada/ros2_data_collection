# Concepts

DC has a small, fixed vocabulary. Using it precisely makes the rest of the documentation
— and the configuration files — unambiguous.

## Glossary

| Term                      | Definition                                                                                                    |
| ------------------------- | ------------------------------------------------------------------------------------------------------------- |
| **Measurement**           | A source of sampled data (CPU, position, camera inspection…) that emits Records.                              |
| **Record**                | One timestamped JSON document flowing through the pipeline.                                                   |
| **Condition**             | A boolean predicate on robot state that gates whether a Measurement's Records are collected.                  |
| **Group**                 | A merge of Records from several Measurements into one Record, based on time proximity.                        |
| **File**                  | A binary artifact produced by a Measurement (image, video, map), uploaded to object storage as-is; only its metadata travels as a Record. |
| **Destination**           | An external system that receives Records or Files (PostgreSQL, S3-compatible storage, console…).              |
| **Blessed Destination**   | A Destination DC configures natively from ROS parameters: `postgres`, `s3`, `file`, `console`.                |
| **Passthrough Destination** | A Destination configured by handing raw Shipper configuration through DC, unlocking the Shipper's full catalog without DC code. |
| **Bridge**                | The DC component (`dc_bridge`) that receives Records from ROS topics and hands them to the Shipper.           |
| **Shipper**               | The external process (Vector by default) that buffers, transforms, and reliably delivers Records to Destinations. |
| **Tag**                   | A label carried by a Record naming the route it is delivered on.                                              |

Relationships between them:

- A **Measurement** emits **Records**, optionally gated by one or more **Conditions**
- A **Group** merges Records from several **Measurements** into one **Record**
- The **Bridge** forwards every **Record** to the **Shipper**
- The **Shipper** delivers Records to one or more **Destinations**
- A **File** is uploaded to an object-storage **Destination**; its metadata becomes a **Record**
- A **Record** carries a **Tag**; each Tag names a route Destinations subscribe to

```admonish example title="Words that mean something specific here"
"Sink" is the Shipper's internal configuration unit, not the DC concept — the DC concept
is **Destination**. "Route" is the Shipper-side path a **Tag** selects. A camera image is
a **File**, not a Record; the Record is the JSON document describing where that File went.
```

## ROS 2

ROS 2 is the core middleware used for DC. If you are unfamiliar with it, visit
[the ROS 2 documentation](https://docs.ros.org/en/rolling/) before continuing.

## Records

A Record is a single data unit in JSON, published on a ROS topic as a
`dc_interfaces/msg/StringStamped` message. For example, a Record from the Memory
Measurement:

```json
{
    "date": "2022-12-04T14:16:06.810999008",
    "memory": {
        "used": 76.007431
    }
}
```

The `StringStamped` message carries:

1. **header**: ROS timestamp as `std_msgs/Header`
2. **data**: the Record, as a JSON string
3. **group_key**: the key this Record is nested under when merged into a Group

The `data` string **must** be valid JSON:

```bash
"Robot_X:1.5, Robot_Y:1.8" # Not a valid Record
"{'x': 1.5, 'y': 1.8}"     # Valid Record
```

### Timestamps

The timestamp is the time the Record was created. Measurements convert ROS time to a
UTC timestamp. Each Destination normalizes it into one field before delivery, controlled
by `time_key` (default `date`) and `time_format` (`double` for Unix epoch seconds, or
`iso8601`).

### JSON validation

Each Record is validated against a JSON schema by default, following the
[JSON Schema Validation](https://json-schema.org/draft/2020-12/json-schema-validation.html)
specification. Each Measurement ships its own schema, which can be overridden from a
custom package or disabled per Measurement with `enable_validator: false`. See
[Data validation](./data_validation.md).

## Bridge and Shipper

DC's data plane is an external **Shipper** process, [Vector](https://vector.dev/), fed
by the **Bridge** (`dc_bridge`) over a local socket. The Bridge renders the Shipper's
entire configuration from plain ROS parameters — see [Destinations](./destinations.md) —
spawns and supervises the Shipper process, and forwards every Record it receives on its
configured input topics.

DC gets the Shipper's disk buffering, backpressure handling, retries, and native
Destination support (PostgreSQL, S3-compatible storage, and many more) without embedding
or forking it (ADR-0001, ADR-0002 in
[`docs/adr/`](https://github.com/minipada/ros2_data_collection/tree/jazzy/docs/adr)).

```admonish info title="Shipper ingest protocol"
The wire format on the local Bridge↔Shipper socket (default port 24224) is Fluentd's
open "Forward" specification — chosen as the cheapest Shipper-native listener with
built-in receipt acknowledgement (ADR-0002). **No Fluentd or Fluent Bit software runs
anywhere in DC 2.0**: the Bridge implements the sender side itself in a few hundred
lines of msgpack. The word "fluent" only appears in the generated Shipper config
(`type = "fluent"`).
```

## Tags

Every Record carries a **Tag**, derived mechanically from the topic it was published on:
the leading `/` is dropped and the remaining `/` become `.`.

| Topic                     | Tag                    | Public Shipper route      |
| ------------------------- | ---------------------- | ------------------------- |
| `/dc/measurement/uptime`  | `dc.measurement.uptime`| `dc.dc.measurement.uptime`|
| `/dc/group/robot`         | `dc.group.robot`       | `dc.dc.group.robot`       |
| *(Uploader-internal)*     | `dc.files`             | `dc.dc.files`             |

Tags are what the Shipper routes on, and the `dc.<tag>` route names are
[stable public API](./destinations.md#the-dctag-routing-contract-public-api) that
passthrough Destinations consume.

```admonish warning title="Tags are not how you select a Destination"
In DC 1.x, a Measurement's `tags` parameter named the destination plugins that should
receive its Records. In DC 2.0 that is inverted: a Destination declares the **topics** it
receives in its own `inputs` list, and Tags are derived, not configured. Delete `tags:`
from Measurement and Group configurations — see the [migration guide](./migration.md#tags-what-changed).
```

## Destinations

A Destination is where data ends up: PostgreSQL, S3-compatible storage, a file, or the
console are **blessed** (rendered from plain ROS parameters); any other Vector sink is
reachable through the **passthrough**. See [Destinations](./destinations.md) for the full
contract.

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]                 # Destination names to enable
    console:                                  # a name you choose
      type: console                           # a blessed type
      receives: records
      inputs: ["/dc/measurement/uptime"]      # the topics this Destination receives

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"  # matched by the Destination's `inputs`
```

## Conditions

A Condition enables or disables one or more Measurements. For example, collect camera
images only when the robot is stopped. A Measurement can require that all, any, or none
of a set of Conditions are active. See [Conditions](./conditions.md).

## Lifecycle Nodes and Bond

*(Source: [Nav2 documentation](https://navigation.ros.org/concepts/index.html))*

Lifecycle (or Managed, more correctly) nodes are unique to ROS 2. More information can be [found here](https://design.ros2.org/articles/node_lifecycle.html). They are nodes that contain state machine transitions for bringup and teardown of ROS 2 servers. This helps in deterministic behavior of ROS systems in startup and shutdown. It also helps users structure their programs in reasonable ways for commercial uses and debugging.

When a node is started, it is in the unconfigured state, only processing the node's constructor which should not contain any ROS networking setup or parameter reading. By the launch system, or the supplied lifecycle manager, the nodes need to be transitioned to inactive by configuring. After, it is possible to activate the node by transitioning through the activating stage.

This state will allow the node to process information and be fully setup to run. The configuration stage, triggering the on_configure() method, will setup all parameters, ROS networking interfaces, and for safety systems, all dynamically allocated memory. The activation stage, triggering the on_activate() method, will active the ROS networking interfaces and set any states in the program to start processing information.

To shutdown, we transition into deactivating, cleaning up, shutting down and end in the finalized state. The networking interfaces are deactivated and stop processing, deallocate memory, exit cleanly, in those stages, respectively.

The lifecycle node framework is used extensively through out this project and all servers utilize it. It is best convention for all ROS systems to use lifecycle nodes if it is possible.

Within DC, we use a wrapper of LifecycleNodes, nav2_util LifecycleNode from Nav2. This wrapper wraps much of the complexities of LifecycleNodes for typical applications. It also includes a bond connection for the lifecycle manager to ensure that after a server transitions up, it also remains active. If a server crashes, it lets the lifecycle manager know and transition down the system to prevent a critical failure. See [Eloquent to Foxy](https://navigation.ros.org/migration/Eloquent.html#eloquent-migration) for details.

```admonish info
The Bridge is deliberately **not** a lifecycle node and **not** under the lifecycle
manager (ADR-0006): it must be up and its Shipper ready *before* the collection nodes
are allowed to activate. See [Data Pipeline](./data_pipeline.md).
```

## Buffering and data persistence

The Shipper manages its own disk buffer (`shipper.data_dir` on the Bridge's parameters)
with a documented minimum size (`shipper.buffer_max_bytes`). This buffer persists across
reboots: Records accepted by the Bridge but not yet delivered to a Destination survive an
outage of that Destination, a robot reboot, or a Bridge restart, and are delivered — with
end-to-end acknowledgements — once the Destination is reachable again.

## Scheduling and retries

The Shipper retries delivery to a Destination on failure with its own backoff,
independent of DC code; see [Vector's documentation](https://vector.dev/docs/) for
details. The Bridge itself is supervised by DC (launch respawn) and, in turn, supervises
its own Shipper child process — including a Linux parent-death signal so the Shipper can
never outlive the Bridge across a crash or SIGKILL.
