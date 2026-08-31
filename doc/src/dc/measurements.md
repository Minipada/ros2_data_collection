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
| custom_key_str_list                          | Custom strings to use in other parameters. They are also appended in the json sent to the destination, and to the File metadata Records of the same Measurement | list\[str\] | N/A                           |
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

### robot_name resolution

`robot_name` is a custom key like any other, but when it appears in `custom_key_str_list`
its value resolves in a fixed order rather than always being a literal string, so a fleet
does not need one hand-edited params file per robot:

1. `custom_keys_str.robot_name.value` — a literal string, unchanged from before.
2. `custom_keys_str.robot_name.value_from_file` — the contents of a file, e.g. one written
   by the provisioning process.
3. The machine's hostname — the default when neither of the above is set.

A `value_from_file` that names a file that cannot be read, or any source that resolves to
an empty string, fails node configuration with a clear error rather than shipping Records
with a missing or blank `robot_name`.

### Custom keys on Files

The keys listed in `custom_key_str_list` label a Measurement's **Files** as well as its
Records: the Bridge's Uploader writes them into the `file_status` and `group_complete`
Records it emits for that Measurement's Files, so both sides of a Destination carry the
same labelling. A Record names its custom keys in a `custom_keys` field for that purpose.

Two limits are worth knowing. A custom key whose name is one the Uploader computes itself
(`group_name`, `local_path`, `remote_path`, `storage_type`, `uploaded`, `size`, …) is not
written — the Uploader's own value is kept and the Bridge logs the collision. The keys the
rows already carry, `robot_name` and `id` (as `robot_id`), are likewise not repeated, and
are not reported: those values are in the row either way. And the column still has to exist
in the Destination: the PostgreSQL sink maps JSON keys onto existing columns 1:1, so a new
custom key needs an `ALTER TABLE` on `dc_files` the same way it needs one on `dc_records`.

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
| **buffer_duration_sec**        | Seconds of history to buffer instead of publishing live; 0 disables buffering and preserves normal live publishing | float | 0 |
| **post_roll_duration_sec**     | Seconds to keep publishing live after a flush, still tagged with the same `incident_id`; 0 means pre-roll only | float | 0 |
| **cooldown_sec**               | Seconds to ignore further `FlushEvent`s once post-roll ends, before buffering re-arms itself; 0 re-arms immediately | float | 0 |
| **max_flush_rate_hz**          | Ceiling on how fast the buffered window is emitted once a flush releases it; 0 releases the whole window in one burst | float | 0 |
| **flush_topic**                | Topic to receive the `FlushEvent` (see [Triggers](./triggers.md)) that releases the buffered window, tagging each Record with the event's `incident_id` | str | "/dc/flush" |

```admonish info title="buffer_duration_sec and friends: pre-event circular-buffer capture"
When `buffer_duration_sec` is set above 0, this Measurement stops publishing live: each
collected sample is instead pushed into an in-memory ring buffer covering the last
`buffer_duration_sec` seconds. A `FlushEvent` on `flush_topic` (published by a `dc_triggers`
broadcast node when its Trigger fires) then drives one incident-capture cycle:

1. **Buffering** — the default, armed state: samples accumulate in the ring buffer and nothing
   is published. Only in this state does a `FlushEvent` start a cycle.
2. **Flushing** — the buffered window is published oldest first, each Record tagged with the
   event's `incident_id` and stamped with when it was *collected*, not when it was released.
   With `max_flush_rate_hz` left at 0 the whole window goes out in one burst; set above 0, it is
   emitted at no more than that many Records per second, so a robot recovering from an incident
   does not also have to absorb the entire window at once. The window is consumed, so the next
   incident releases its own history rather than replaying this one. Samples collected while a
   rate-limited release is still draining are buffered, not published, and so become part of the
   *next* incident's pre-roll.
3. **PostRoll** — for `post_roll_duration_sec` after the release finishes, samples are published
   live as they are collected, still tagged with the same `incident_id`, so the aftermath of the
   incident is captured too. It runs from the end of the release, not from the `FlushEvent`, so
   a rate-limited release does not eat into it. Left at its default 0, this phase is skipped
   entirely: pre-roll only.
4. **Cooldown** — for `cooldown_sec` after post-roll ends, further `FlushEvent`s are ignored, so
   a flapping Trigger cannot produce a flood of overlapping incidents. Samples are buffered
   again during this phase, so the next incident still gets a full pre-roll window.

Files follow their Records. A Measurement that produces Files (camera, map, …) normally leaves
them under `save_local_base_path` for the Bridge to upload as soon as the Record naming them is
published — but an armed Measurement publishes nothing, so every File it produces while buffering
(also during cooldown, and while a rate-limited release is still draining) is instead *moved* into
a scratch directory beside the save path,
`<save_local_base_path>/.dc_incident_scratch/<measurement_name>/`, and the buffered Record is
rewritten to reference the staged copy. That scratch directory rolls on the same
`buffer_duration_sec` window as the Records themselves: a staged File is deleted from disk at the
same moment its Record ages out of the ring buffer, so an armed Measurement's Files stay bounded
instead of accumulating images no Record will ever carry to the Bridge. On release the staged
copies are handed on with the Records referencing them — `remote_paths` is untouched, so each File
uploads to exactly the Destination key it was collected under — and the scratch ring stops
tracking them, leaving the Bridge's usual retention sweep and `delete_when_sent` to clean them up.
The Bridge needs no configuration for any of this: a released Record is an ordinary Files Record
that happens to be older than usual. Files collected during **PostRoll** are published live and
never staged at all.

The Measurement then re-arms itself back to **Buffering** with no manual intervention — a
second incident is captured exactly like the first. With both `post_roll_duration_sec` and
`cooldown_sec` left at 0, a flush releases the pre-roll window and the Measurement is armed
again immediately.

`incident_id` is a top-level field of the Record envelope, beside `tags`, `run_id` and `name`
— not a key nested inside the measurement's own data — so a `postgres` Destination stores it
in its own `incident_id` column and "everything from this one event" is a plain
`WHERE incident_id = '…'`. See [Destinations](./destinations.md#incident_id) for the column
the table needs. A Record collected outside an incident carries no `incident_id` at all,
leaving the column NULL. A [Group](./groups.md) lifts a member's `incident_id` onto the merged
Record the same way it does `tags`, so grouping does not bury it.
```

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
| [Battery](./measurements/battery.md)                     | Charge percentage, voltage and current of a pack, plus charging sessions and completed cycles            |
| [Camera](./measurements/camera.md)                       | Camera images, images can be rotated and inspected to detect content in images. They are saved as files |
| [Command velocity](./measurements/cmd_vel.md)            | Command velocity: navigation commands                                                                   |
| [CPU](./measurements/cpu.md)                             | CPU statistics                                                                                          |
| [Distance traveled](./measurements/distance_traveled.md) | Total distance traveled by the robot                                                                    |
| [Dummy](./measurements/dummy.md)                         | Dummy event, for testing and debugging                                                                  |
| [Fast DDS statistics](./measurements/fastdds_stats.md)   | eProsima Fast DDS's own Statistics Module: latency, throughput, RTPS packets, physical-layer data. Fast-DDS-specific |
| [Fault](./measurements/fault.md)                         | Component diagnostic level transitions: one Record per raise, change or clear, a source for MTBF/MTTR   |
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
