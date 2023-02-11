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

The node starts the fluent bit engine and its ros2 plugin and enables data collection from ROS2 topics. This plugin will subscribe to the configured ROS2 topics and data will be collected by Fluent bit to destinations enabled by the [destination node](./destinations.md).
Each topic is configured in a measurement, which is loaded in this node with pluginlib.
In addition, conditions are pluginlibs plugin also loaded dynamically. They are optional plugins that allow to collect on some conditions, e.g robot is moving.

| Parameter name       | Description                                                                                                                        | Type(s)     | Default         |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------------------- | ----------- | --------------- |
| measurement_plugins  | Name of the measurement plugins to load                                                                                            | list\[str\] | N/A (mandatory) |
| condition_plugins    | Name of the condition plugins to load                                                                                              | list\[str\] | N/A (mandatory) |
| save_local_base_path | Path where files will be saved locally (e.g camera images). Expands $X to environment variables and =Y to custom string parameters | str         | N/A (mandatory) |
| all_base_path        | Path where files will be saved at their destination (S3, minio...). Expands $X to environment variables and =Y to                  | str         | N/A (mandatory) |
| custom_str_params    | Custom string parameters that can be used in other parameters.                                                                     | list\[str\] | N/A (optional)  |


## Plugin parameters

Each measurement is collected through a node and has these configuration parameters:

| Parameter name             | Description                                                                                  | Type(s)     | Default         |
| -------------------------- | -------------------------------------------------------------------------------------------- | ----------- | --------------- |
| plugin                     | Name of the plugin to load                                                                   | str         | N/A (mandatory) |
| topic_output               | Topic where result will be published                                                         | str         | N/A (mandatory) |
| group_key                  | Value of the key used when grouped                                                           | str         | N/A (mandatory) |
| polling_interval           | Interval to which data is collected in milliseconds                                          | int (>=100) | 1000            |
| init_collect               | Collect when the node starts instead of waiting the first tick                               | bool        | true            |
| init_max_measurements      | Collect a maximum of n measurements when starting the node (-1 = never, 0 = infinite)        | int         | 0               |
| condition_max_measurements | Collect a maximum of n measurements when conditions are activated (-1 = never, 0 = infinite) | int         | 0               |
| enable_validator           | Will validate the data against a JSON schema                                                 | bool        | false           |
| remote_prefixes            | Prefixes to apply to the paths when sending files to a destination                           | str         | ""              |
| remote_keys                | Remote keys                                                                                  | str         | ""              |
| if_all_conditions          | Collect only if all conditions are activated                                                 | list\[str\] | N/A (optional)  |
| if_any_conditions          | Collect if any conditions is activated                                                       | list\[str\] | N/A (optional)  |
| if_none_conditions         | Collect only if all conditions are not activated                                             | list\[str\] | N/A (optional)  |

## Available plugins:

| Name                                                     | Description                                                                                             |
| -------------------------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| [Camera](./measurements/camera.md)                       | Camera images, images can be rotated and inspected to detect content in images. They are saved as files |
| [Command velocity](./measurements/cmd_vel.md)            | Command velocity: navigation commands                                                                   |
| [CPU](./measurements/cpu.md)                             | CPU statistics                                                                                          |
| [Distance traveled](./measurements/distance_traveled.md) | Total distance traveled by the robot                                                                    |
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
| [Uptime](./measurements/uptime.md)                       | How long the machine has been turned on                                                                 |

## Example configuration

```yaml
measurement_server:
  ros__parameters:
    measurement_plugins: [
        "camera",
        "memory",
        "cpu",
        "uptime",
        "storage",
        "permission_home",
        "cmd_vel",
        "position",
        "speed",
        "map",
        "uptime",
    ]
    condition_plugins: ["moving"]
    custom_str_params: ["robot_name"]
    save_local_base_path: "$HOME/windrose/src/windrose/data/"
    all_base_path: "=fleet_name/=robot_name/%Y/%m/%d/%H"
    not_loaded_param: "a_param" # Not being used because not in custom_str_params
    robot_name: "r2d2"
    moving:
      plugin: "dc_conditions/Moving"
    camera:
      plugin: "dc_measurements/Camera"
      group_key: "camera"
      topic_output: "/dc/measurement/camera"
      polling_interval: 1000
      init_collect: true
      node_name: "dc_measurement_camera"
      cam_topic: "/camera/image_raw"
      cam_name: my_cam
      enable_validator: false
      draw_det_barcodes: false
      save_raw_img: true
      save_rotated_img: true
      save_detections_img: true
      save_raw_path: "camera/raw/%Y-%m-%dT%H:%M:%S"
      save_rotated_path: "camera/rotated/%Y-%m-%dT%H:%M:%S"
      save_inspected_path: "camera/inspected/%Y-%m-%dT%H:%M:%S"
      rotation_angle: 0
      detection_modules: ["barcode"]
      remote_prefixes: [""]
      remote_keys: ["minio"]
    cpu:
      plugin: "dc_measurements/Cpu"
      group_key: "cpu"
      topic_output: "/dc/measurement/cpu"
      polling_interval: 2000
      init_collect: True
      node_name: "dc_measurement_cpu"
      cpu_min: 5
      max_processes: 5
    memory:
      plugin: "dc_measurements/Memory"
      group_key: "memory"
      topic_output: "/dc/measurement/memory"
      polling_interval: 1500
      init_collect: False
      node_name: "dc_measurement_memory"
    uptime:
      plugin: "dc_measurements/Uptime"
      group_key: "uptime"
      topic_output: "/dc/measurement/uptime"
    storage:
      plugin: "dc_measurements/Storage"
      group_key: "storage"
      env: HOME
      topic_output: "/dc/measurement/storage"
    permission_home:
      plugin: "dc_measurements/Permissions"
      topic_output: "/dc/measurement/permission_home"
      group_key: "permission_home"
      env: HOME
      permission_format: 1
    cmd_vel:
      plugin: "dc_measurements/CmdVel"
      group_key: "cmd_vel"
      enable_validator: true
      topic_output: "/dc/measurement/cmd_vel"
    os:
      plugin: "dc_measurements/OS"
      group_key: "os"
      topic_output: "/dc/measurement/os"
    network: # Not loaded because not in measurement_plugins
      plugin: "dc_measurements/Network"
      group_key: "network"
      topic_output: "/dc/measurement/network"
    map:
      plugin: "dc_measurements/Map"
      group_key: "map"
      save_map_path: "map/%Y-%m-%dT%H:%M:%S"
      topic_output: "/dc/measurement/map"
    position:
      plugin: "dc_measurements/Position"
      group_key: "position"
      global_frame: "map"
      robot_base_frame: "base_link"
      transform_tolerance: 0.1
      topic_output: "/dc/measurement/position"
    speed:
      plugin: "dc_measurements/Speed"
      group_key: "speed"
      odom_topic: "/odom"
      topic_output: "/dc/measurement/speed"
```
