# Concepts

There are a few key concepts that are really important to understand how DC operates.

## ROS 2

ROS 2 is the core middleware used for DC. If you are unfamilar with this, please visit [the ROS 2 documentation](https://docs.ros.org/en/rolling/) before continuing.

## Fluent Bit

Fluent Bit is used in the backend for most plugins in DC. If you are unfamilar with this, please visit [the Fluent Bit documentation](https://docs.fluentbit.io/manual/)

The DC destination ROS node starts it using the fluent bit C api and thus DC directly gets all benefits from it.

Fluent Bit configuration has been wrapped in this node and all its configuration parameters can be passed from the YAML configuration file.

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

Internally, when using Fluent Bit based plugins, it will contain 2 components: its timestamp and its message. For us, in DC, it will always be a JSON string sent in a ROS message of StringStamped type. This ROS message contains:

1. **header**: ROS timestamp as std_msgs/Header
2. **data**: JSON message as string
3. **group_key**: a string used as a key for the new message when grouping multiple messages together

## Tag(s)
Every measurement requires to have at least a tag configured (via the `tags` parameter) so it is sent to its destination(s). This tag corresponds to the name of the plugin you defined in the same configuration. It is then used in a later stage by the Router to decide which Filter or Output phase it must go through.

Example:

```yaml
destination_server:
  ros__parameters:
    destination_plugins: ["flb_stdout"]
    flb_stdout: # Custom name for the plugin
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/measurement/string_stamped"]
measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
      tags: ["flb_stdout"] # Match the plugin set in the destination_server
```

To manage the tags in DC, we pass the tags as parameter to each measurement and automatically use 2 Fluent Bit filters to assign the ROS message to a certain Fluent Bit output:

1. rewrite_tag filter: Modify the message tag to the destination(s) configured
2. lua filter: Take the flags received as a string containing a list and set the tags internally in Fluent Bit.

You can find th code in flb_destination.hpp

## Match
Fluent Bit allows to deliver your collected and processed Events to one or multiple destinations, this is done through a routing phase. A Match represent a simple rule to select Events where its tags matches a defined rule.

## Destinations
A destination is where the data will be sent: AWS S3, stdout, AWS Kinesis. It has the possibility to use [outputs from fluentbit](https://docs.fluentbit.io/manual/pipeline/outputs).

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

Fluent Bit has its own buffering management, explained in [its documentation](https://docs.fluentbit.io/manual/concepts/buffering). Data can be stored in Memory or/and filesystem.

By default, DC uses the memory buffering for a small amount of data (5M) and then uses filesystem buffering.

The configuration is printed when starting the destination server:

```bash
[destination_server-2] [2023/02/24 10:36:15] [ info] [storage] ver=1.3.0, type=memory+filesystem, sync=full, checksum=off, max_chunks_up=128
[destination_server-2] [2023/02/24 10:36:15] [ info] [input:ros2:ros2.0] storage_strategy='filesystem' (memory + filesystem)
[destination_server-2] [2023/02/24 10:36:15] [ info] [input:storage_backlog:storage_backlog.1] queue memory limit: 976.6K
```

This buffering ensures data will persist across reboots

## Scheduling and Retries

DC inherits from Fluent Bit features of scheduling and retries. It can be configured in the destination node. More about it can be read on the [Fluent Bit documentation](https://docs.fluentbit.io/manual/administration/scheduling-and-retries)
