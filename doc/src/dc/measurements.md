# Measurements
## Description
Measurements are a single data unit presented in JSON format, that can contain different fields. For example, Memory measurement:

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

The node starts the fluent bit engine and its ros2 plugin and enables data collection from ROS 2 topics. This plugin will subscribe to the configured ROS 2 topics and data will be collected by Fluent bit to destinations enabled by the [destination node](./destinations.md).
Each topic is configured in a measurement, which is loaded in this node with pluginlib.
In addition, conditions are pluginlibs plugin also loaded dynamically. They are optional plugins that allow to collect on some conditions, e.g robot is moving.

| Parameter name       | Description                                                                                                                        | Type(s)     | Default                       |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------------------- | ----------- | ----------------------------- |
| measurement_plugins  | Name of the measurement plugins to load                                                                                            | list\[str\] | N/A (mandatory)               |
| condition_plugins    | Name of the condition plugins to load                                                                                              | list\[str\] | N/A (mandatory)               |
| save_local_base_path | Path where files will be saved locally (e.g camera images). Expands $X to environment variables and =Y to custom string parameters | str         | "$HOME/ros2/data/%Y/%M/%D/%H" |
| all_base_path        | Path where files will be saved at their destination (S3, minio...). Expands $X to environment variables and =Y to                  | str         | ""                            |
| custom_str_params    | Custom string parameters that can be used in other parameters.                                                                     | list\[str\] | N/A (optional)                |
| run_id.enabled       | Identify which run the robot is. A new one is generated at every start of the node. Uses either a counter or UUID                  | str         | true                          |
| run_id.counter       | Enable counter for the run_id                                                                                                      | str         | true                          |
| run_id.counter_path  | Path to store the last run. It is expanded with environment variables id                                                           | str         | "$HOME/run_id"                |
| run_id.uuid          | Generate a new run ID by using a random UUID                                                                                       | str         | false                         |

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
| **tags**                       | Tags used by Fluent Bit to do the matching to destinations                                   | list\[str\] | N/A (mandatory)                      |
| **remote_prefixes**            | Prefixes to apply to the paths when sending files to a destination                           | str         | N/A (optional)                       |
| **remote_keys**                | Used by some plugins to generate remote paths                                                | list\[str\] | N/A (optional)                       |
| **if_all_conditions**          | Collect only if all conditions are activated                                                 | list\[str\] | N/A (optional)                       |
| **if_any_conditions**          | Collect if any conditions is activated                                                       | list\[str\] | N/A (optional)                       |
| **if_none_conditions**         | Collect only if all conditions are not activated                                             | list\[str\] | N/A (optional)                       |
| **include_measurement_name**   | Include measurement name in the JSON data                                                    | bool        | false                                |
| **include_measurement_plugin** | Include measurement plugin name in the JSON data                                             | bool        | false                                |

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
