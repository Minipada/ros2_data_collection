# Concepts

There are a few key concepts that are really important to understand how DC operates.

## ROS 2

ROS 2 is the core middleware used for DC. If you are unfamilar with this, please visit [the ROS 2 documentation](https://docs.ros.org/en/rolling/) before continuing.

## Shipper (Vector)

DC's data plane is an external **Shipper** process, [Vector](https://vector.dev/),
fed over the Fluent Forward protocol by the **Bridge** (`dc_bridge`). The Bridge
renders Vector's configuration from plain ROS parameters — see
[Destinations](./destinations.md) — spawns and supervises the Vector process, and
forwards every Record it receives on its configured input topics. DC gets Vector's
disk buffering, retries, and native sinks (PostgreSQL, S3-compatible storage, and
many more) without embedding or forking the Shipper itself (ADR-0001,
`docs/adr/`).

## Lifecycle Nodes and Bond
*(Source: [Nav2 documentation](https://navigation.ros.org/concepts/index.html))*

Lifecycle (or Managed, more correctly) nodes are unique to ROS 2. More information can be [found here](https://design.ros2.org/articles/node_lifecycle.html). They are nodes that contain state machine transitions for bringup and teardown of ROS 2 servers. This helps in deterministic behavior of ROS systems in startup and shutdown. It also helps users structure their programs in reasonable ways for commercial uses and debugging.

When a node is started, it is in the unconfigured state, only processing the node’s constructor which should not contain any ROS networking setup or parameter reading. By the launch system, or the supplied lifecycle manager, the nodes need to be transitioned to inactive by configuring. After, it is possible to activate the node by transitioning through the activating stage.

This state will allow the node to process information and be fully setup to run. The configuration stage, triggering the on_configure() method, will setup all parameters, ROS networking interfaces, and for safety systems, all dynamically allocated memory. The activation stage, triggering the on_activate() method, will active the ROS networking interfaces and set any states in the program to start processing information.

To shutdown, we transition into deactivating, cleaning up, shutting down and end in the finalized state. The networking interfaces are deactivated and stop processing, deallocate memory, exit cleanly, in those stages, respectively.

The lifecycle node framework is used extensively through out this project and all servers utilize it. It is best convention for all ROS systems to use lifecycle nodes if it is possible.

Within DC, we use a wrapper of LifecycleNodes, nav2_util LifecycleNode from Nav2. This wrapper wraps much of the complexities of LifecycleNodes for typical applications. It also includes a bond connection for the lifecycle manager to ensure that after a server transitions up, it also remains active. If a server crashes, it lets the lifecycle manager know and transition down the system to prevent a critical failure. See [Eloquent to Foxy](https://navigation.ros.org/migration/Eloquent.html#eloquent-migration) for details.

## Measurements

Measurements are a single data unit presented in JSON format, that can contain different fields. For example, Memory measurement:

```json
{
    "date": "2022-12-04T14:16:06.810999008",
    "memory": {
        "used": 76.007431
    },
}
```

Every incoming piece of data that belongs to a log or a metric that is retrieved by DC is considered an Event or Record.

Internally, it will always be a JSON string sent in a ROS message of StringStamped type. This ROS message contains:

1. **header**: ROS timestamp as std_msgs/Header
2. **data**: JSON message as string
3. **group_key**: a string used as a key for the new message when grouping multiple messages together

## Tag(s)
Every measurement requires to have at least a tag configured (via the `tags` parameter) so it is sent to its destination(s). This tag corresponds to the name of a Destination declared on the Bridge (`dc_bridge`) in the same configuration.

Example:

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console: # Custom name for the Destination
      type: console
      receives: records
      inputs: ["/dc/measurement/string_stamped"]
measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
      tags: ["console"] # Match the Destination name set on the Bridge
```

See [Destinations](./destinations.md) for the full config renderer contract, including
`inputs`/`tags` matching and the `dc.<tag>` routing convention.

## Destinations
A destination is where the data will be sent: PostgreSQL, S3-compatible storage, a
file, or the console are blessed (rendered from plain ROS parameters); any other
Vector sink is reachable through the passthrough. See [Destinations](./destinations.md)
for the full contract.

## Conditions
A condition enables or disables one or multiple measurements to be published and thus collected. We could for example enable collecting camera images only when a robot is stopped.

Data collection for a measurement can be enabled if one of many conditions are activated, multiple conditions are activated or none.

## Timestamp

The Timestamp represents the time when an Event was created. All events are converting the ROS now time to timestamps (UTC)

## JSON Messages

In DC, all messages sent by measurements are a ROS message and the data string **must** be a JSON message.

```bash
"Robot_X:1.5, Robot_Y:1.8" # Not valid data string
"{'x': 1.5, 'y': 1.8}"     # Valid data string
```

## JSON validation

Each record follows a JSON schema by default, it follows this specification document [JSON Schema Validation](https://json-schema.org/draft/2020-12/json-schema-validation.html).

Each measurement has its own JSON schema, which can be overwritten in a custom package or disabled.

## Buffering and data persistence

Vector, the Shipper, manages its own disk buffer (`shipper.data_dir` on the Bridge's
parameters) with a documented minimum size (`shipper.buffer_max_bytes`). This buffer
persists across reboots: Records accepted by the Bridge but not yet delivered to a
Destination survive an outage of that Destination, a robot reboot, or a Bridge
restart, and are delivered — with end-to-end acknowledgements — once the Destination
is reachable again.

## Scheduling and Retries

Vector retries delivery to a Destination on failure with its own backoff, independent
of DC code; see [its documentation](https://vector.dev/docs/) for details. The Bridge
itself is supervised by DC (launch respawn) and, in turn, supervises its own Vector
child process — including a Linux parent-death signal so Vector can never outlive the
Bridge across a crash or SIGKILL.
