# Measurements

## Description

A **Measurement** is a source of sampled data that emits **Records** — timestamped JSON
documents — on its own ROS topic. For example, a Record from the Memory Measurement:

```json
{
    "date": "2022-12-04T14:16:06.810999008",
    "memory": {
        "used": 76.007431
    },
    "id": "3c70afdcb6f248f28f4c3980734064c5",
    "robot_name": "C3PO",
    "run_id": 358
}
```

## Node parameters

This node collects data and publishes it as Records. Each Measurement is a pluginlib
plugin loaded into this node and publishes on its own `topic_output`; the Bridge
(`dc_bridge`) subscribes to those topics and forwards the Records to the Destinations
that list them in `inputs` (see [Destinations](./destinations.md)). **Conditions** are
pluginlib plugins loaded here too — optional predicates that gate whether a Measurement
collects, e.g. only when the robot is not moving.

```admonish warning title="tags no longer selects a Destination"
In DC 1.x, a Measurement's `tags` parameter named the destination plugins that should
receive its Records. In DC 2.0, a Destination declares the topics it receives in its own
`inputs` list. The parameter is still read, and a non-empty value is still written into
the Record as a `tags` field, but it has no routing effect — remove it. See the
[migration guide](./migration.md#tags-what-changed).
```

| Parameter name                               | Description                                                                                                                                               | Type(s)     | Default                       |
| -------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------- | ----------------------------- |
| measurement_plugins                          | Name of the measurement plugins to load                                                                                                                   | list\[str\] | N/A (mandatory)               |
| condition_plugins                            | Name of the condition plugins to load                                                                                                                     | list\[str\] | N/A (mandatory)               |
| save_local_base_path                         | Path where files will be saved locally (e.g camera images). Expands $X to environment variables and =Y to custom string parameters                        | str         | "$HOME/ros2/data/%Y/%M/%D/%H" |
| all_base_path                                | Path where files will be saved at their destination (S3, RustFS...). Expands $X to environment variables and =Y to custom string parameters               | str         | ""                            |
| custom_key_str_list                          | Custom strings to use in other parameters. They are also appended in the json sent to the destination                                                     | list\[str\] | N/A                           |
| custom_keys_str.force_override               | Override values if the keys are already present in the measurement. Applies to all and can be overridden by `custom_keys_str.<param_name>.force_override` | bool        | false                         |
|                                              |                                                                                                                                                           |             |                               |
| custom_keys_str.<param_name>.name            | Key to add in the serialized data                                                                                                                         | str         | N/A (optional)                |
| custom_keys_str.<param_name>.value           | Value to set for the key as a fixed string                                                                                                                | str         | N/A (optional)                |
| custom_keys_str.<param_name>.value_from_file | Path to a file containing the value to set                                                                                                                | str         | N/A (optional)                |
| custom_keys_str.<param_name>.force_override  | Override value if the key is already present in the measurement                                                                                           | bool        | false                         |
| run_id.enabled                               | Identify which run the robot is. A new one is generated at every start of the node. Uses either a counter or UUID                                         | str         | true                          |
| run_id.counter                               | Enable counter for the run_id                                                                                                                             | str         | true                          |
| run_id.counter_path                          | Path to store the last run. It is expanded with environment variables id                                                                                  | str         | "$HOME/run_id"                |
| run_id.uuid                                  | Generate a new run ID by using a random UUID                                                                                                              | str         | false                         |

## Plugin parameters

Each measurement is collected through a node and has these configuration parameters:

| Parameter name                 | Description                                                                                  | Type(s)     | Default                              |
| ------------------------------ | -------------------------------------------------------------------------------------------- | ----------- | ------------------------------------ |
| **plugin**                     | Name of the plugin to load                                                                   | str         | N/A (mandatory)                      |
| **topic_output**               | Topic where result will be published                                                         | str         | "/dc/measurement/<measurement_name>" |
| **group_key**                  | Value of the key used when grouped                                                           | str         | N/A (mandatory)                      |
| **debug**                      | More verbose output                                                                          | bool        | false                                |
| **polling_interval**           | Interval to which data is collected in milliseconds                                          | int (>=100) | 1000                                 |
| **init_collect**               | Collect when the node starts instead of waiting the first tick                               | bool        | true                                 |
| **init_max_measurements**      | Collect a maximum of n measurements when starting the node (-1 = never, 0 = infinite)        | int         | 0                                    |
| **condition_max_measurements** | Collect a maximum of n measurements when conditions are activated (-1 = never, 0 = infinite) | int         | 0                                    |
| **enable_validator**           | Will validate the data against a JSON schema                                                 | bool        | true                                 |
| **json_schema_path**           | Path to the JSON schema, ignored if empty string                                             | str         | N/A (optional)                       |
| **remote_prefixes**            | Prefixes to apply to the remote paths of the Files this Measurement produces                 | str         | N/A (optional)                       |
| **remote_keys**                | Destination names the Files this Measurement produces are uploaded to; each becomes a key under the Record's `remote_paths` | list\[str\] | N/A (optional) |
| **if_all_conditions**          | Collect only if all conditions are activated                                                 | list\[str\] | N/A (optional)                       |
| **if_any_conditions**          | Collect if any conditions is activated                                                       | list\[str\] | N/A (optional)                       |
| **if_none_conditions**         | Collect only if all conditions are not activated                                             | list\[str\] | N/A (optional)                       |
| **gate_condition**             | Name of a Condition that must become true once before any collection is published; then latches open permanently and is never consulted again | str | N/A (optional) |
| **include_measurement_name**   | Include measurement name in the JSON data                                                    | bool        | false                                |
| **include_measurement_plugin** | Include measurement plugin name in the JSON data                                             | bool        | false                                |

```admonish info title="gate_condition vs. if_all/if_any/if_none_conditions"
`gate_condition` is a one-shot arming latch, not a per-collection gate: it names a single
Condition plugin (any type under `dc_measurements/plugins/conditions/`) that suppresses
**every** collection — including the `init_collect` Record normally published on
activation — until that Condition becomes true for the first time. Once armed, the
Condition is never consulted again for the lifetime of the node, even if it later becomes
false again; re-arming does not happen. This is unlike `if_all_conditions`/
`if_any_conditions`/`if_none_conditions`, which are re-evaluated on every collection and can
suppress publishing again once their Conditions change. If the named Condition doesn't
exist among `condition_plugins`, collection is held back permanently and an error is
logged.
```

```admonish info title="How if_all/if_any/if_none_conditions combine"
The three lists are evaluated on every collection and ANDed together: `if_all_conditions`
objects unless every Condition it names is active, `if_any_conditions` unless at least one
of its Conditions is active, and `if_none_conditions` unless every Condition it names is
inactive. A list left empty never objects, so a Measurement naming no Condition at all
always collects. A name that is not among `condition_plugins` reads as inactive and an
error is logged: it blocks collection when listed in `if_all_conditions` or
`if_any_conditions`, and is accepted by `if_none_conditions`.
```

## Available plugins:

| Name                                                     | Description                                                                                             |
| -------------------------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| [Camera](./measurements/camera.md)                       | Camera images, images can be rotated and inspected to detect content in images. They are saved as files |
| [Command velocity](./measurements/cmd_vel.md)            | Command velocity: navigation commands                                                                   |
| [CPU](./measurements/cpu.md)                             | CPU statistics                                                                                          |
| [Distance traveled](./measurements/distance_traveled.md) | Total distance traveled by the robot                                                                    |
| [Dummy](./measurements/dummy.md)                         | Dummy event, for testing and debugging                                                                  |
| [IP Camera](./measurements/ip_camera.md)                 | IP camera videos as files                                                                               |
| [Map](./measurements/map.md)                             | ROS map files (yaml and pgm) and metadata used by the robot to localize and navigate                    |
| [Memory](./measurements/memory.md)                       | System memory usage                                                                                     |
| [Network](./measurements/network.md)                     | Network interfaces, availability                                                                        |
| [OS](./measurements/os.md)                               | Operating System information                                                                            |
| [Permissions](./measurements/permissions.md)             | Permissions of a file or directory                                                                      |
| [Position](./measurements/position.md)                   | Robot position                                                                                          |
| [Speed](./measurements/speed.md)                         | Robot speed                                                                                             |
| [Storage](./measurements/storage.md)                     | Available and used space in a directory                                                                 |
| [String stamped](./measurements/string_stamped.md)       | Republish a string stamped message, can be used for external data                                       |
| [TCP Health](./measurements/tcp_health.md)               | Health status of a TCP Server                                                                           |
| [Uptime](./measurements/uptime.md)                       | How long the machine has been turned on                                                                 |
