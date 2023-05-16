# Turtlebot3

In this example, we add a robot and start collecting robot data to Stdout.

You will also need 2 terminal windows, to:

1. Run the Nav2 turtlebot3 launchfile: it starts localization, navigation and RViz
2. Run DC

Since RViz is pretty verbose, using 2 terminal windows will help reading the JSON printed on the terminal window.

## Setup the environment

In each, terminal, source your environment and setup turtlebot configuration:

```bash
$ source /opt/ros/humble/setup.bash
$ source install/setup.bash
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
$ export TURTLEBOT3_MODEL=waffle
```

## Start Navigation

Then, start the Turtlebot launchfile:

```bash
$ ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

RViz and Gazebo will start: you should now see the robot in Gazebo, and the map on RViz.

Set the robot position using the "2D Pose Estimate" button.

```admonish info
If any problem occur, please take a look at the [nav2 official documentation](https://navigation.ros.org/getting_started/index.html#running-the-example) which covers the case.
```

## Start DC

Execute

```bash
$ ros2 launch dc_demos tb3_simulation_stdout.launch.py
```

At the end, the data is displayed:

```
[component_container_isolated-1] [{"date":1677690777.96911,"width":384,"height":384,"remote_paths":{"minio":{"pgm":"C3PO/2023/03/01/17/map/2023-03-01T17:12:57.pgm","yaml":"C3PO/2023/03/01/17/map/2023-03-01T17:12:57.yaml"}},"resolution":0.05000000074505806,"origin":{"x":-10,"y":-10},"local_paths":{"pgm":"/root/dc_data/C3PO/2023/03/01/17/map/2023-03-01T17:12:57.pgm","yaml":"/root/dc_data/C3PO/2023/03/01/17/map/2023-03-01T17:12:57.yaml"},"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}]
[component_container_isolated-1] [{"date":1677690782.883036,"width":384,"height":384,"remote_paths":{"minio":{"pgm":"C3PO/2023/03/01/17/map/2023-03-01T17:13:02.pgm","yaml":"C3PO/2023/03/01/17/map/2023-03-01T17:13:02.yaml"}},"resolution":0.05000000074505806,"origin":{"x":-10,"y":-10},"local_paths":{"pgm":"/root/dc_data/C3PO/2023/03/01/17/map/2023-03-01T17:13:02.pgm","yaml":"/root/dc_data/C3PO/2023/03/01/17/map/2023-03-01T17:13:02.yaml"},"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}]
[component_container_isolated-1] [{"position":{"x":-0.7254206029057992,"yaw":0.134098723062189,"tags":["flb_stdout"],"y":-0.5116378019627142},"cmd_vel":{"tags":["flb_stdout"],"linear":{"x":0.26,"z":0,"y":0},"angular":{"x":0,"z":-0.157895,"y":0}},"speed":{"tags":["flb_stdout"],"linear":{"x":8.720555295508514e-05,"z":0,"y":4.219923511090644e-06},"computed":8.730759543499989e-05,"angular":{"x":-0.0002958005596118955}},"date":1677690787.878526,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}]
[component_container_isolated-1] [{"date":1677690787.883072,"width":384,"height":384,"remote_paths":{"minio":{"pgm":"C3PO/2023/03/01/17/map/2023-03-01T17:13:07.pgm","yaml":"C3PO/2023/03/01/17/map/2023-03-01T17:13:07.yaml"}},"resolution":0.05000000074505806,"origin":{"x":-10,"y":-10},"local_paths":{"pgm":"/root/dc_data/C3PO/2023/03/01/17/map/2023-03-01T17:13:07.pgm","yaml":"/root/dc_data/C3PO/2023/03/01/17/map/2023-03-01T17:13:07.yaml"},"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}]
[component_container_isolated-1] [{"position":{"x":-0.1931822640325796,"yaw":-0.08553498481283986,"tags":["flb_stdout"],"y":-0.5267276846092974},"cmd_vel":{"tags":["flb_stdout"],"linear":{"x":0.232632,"z":0,"y":0},"angular":{"x":0,"z":-0.0526316,"y":0}},"speed":{"tags":["flb_stdout"],"linear":{"x":0.2466804589428482,"z":0,"y":4.147463233418404e-05},"computed":0.2466804624294339,"angular":{"x":-0.1575310923819759}},"date":1677690789.878385,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}]
[component_container_isolated-1] [{"position":{"x":-0.06732902212346159,"yaw":-0.06199357549188581,"tags":["flb_stdout"],"y":-0.5350783704643411},"cmd_vel":{"tags":["flb_stdout"],"linear":{"x":0.191579,"z":0,"y":0},"angular":{"x":0,"z":-0.0526316,"y":0}},"speed":{"tags":["flb_stdout"],"linear":{"x":0.2464715530337069,"z":0,"y":1.735682541426087e-05},"computed":0.2464715536448513,"angular":{"x":-0.05272644093984532}},"date":1677690790.878357,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}]
[component_container_isolated-1] [{"position":{"x":0.1426837169393564,"yaw":-0.1292500346335131,"tags":["flb_stdout"],"y":-0.5883037844577782},"cmd_vel":{"tags":["flb_stdout"],"linear":{"x":0,"z":0,"y":0},"angular":{"x":0,"z":0,"y":0}},"speed":{"tags":["flb_stdout"],"linear":{"x":0.1917414358977791,"z":0,"y":1.44170547699829e-05},"computed":0.1917414364397889,"angular":{"x":-0.05280628215810287}},"date":1677690791.878987,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}]
[component_container_isolated-1] [{"position":{"x":0.2658444927274801,"yaw":-0.1669572018597313,"tags":["flb_stdout"],"y":-0.6121168728850158},"cmd_vel":{"tags":["flb_stdout"],"linear":{"x":0,"z":0,"y":0},"angular":{"x":0,"z":0,"y":0}},"speed":{"tags":["flb_stdout"],"linear":{"x":3.645119603853573e-05,"z":0,"y":3.14134397110158e-06},"computed":3.658630528742332e-05,"angular":{"x":-0.0004012554003809027}},"date":1677690792.879082,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}]
[component_container_isolated-1] [{"date":1677690792.883126,"width":384,"height":384,"remote_paths":{"minio":{"pgm":"C3PO/2023/03/01/17/map/2023-03-01T17:13:12.pgm","yaml":"C3PO/2023/03/01/17/map/2023-03-01T17:13:12.yaml"}},"resolution":0.05000000074505806,"origin":{"x":-10,"y":-10},"local_paths":{"pgm":"/root/dc_data/C3PO/2023/03/01/17/map/2023-03-01T17:13:12.pgm","yaml":"/root/dc_data/C3PO/2023/03/01/17/map/2023-03-01T17:13:12.yaml"},"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}
```

Given the JSON is quite large, let's analyze 2 different records:

The first one being the data published on the robot group:
```json
[
  {
    "cmd_vel": {
      "linear": {
        "x": 0.246316,
        "y": 0,
        "z": 0
      },
      "angular": {
        "x": 0,
        "y": 0,
        "z": 0.263158
      }
    },
    "date": 1677694799.685468,
    "position": {
      "x": -0.6033772358727438,
      "yaw": -0.7146495585355921,
      "y": -1.633862970534114
    },
    "speed": {
      "angular": {
        "x": -0.0002542699063698451
      },
      "computed": 3.279051750818494e-05,
      "linear": {
        "x": 3.244397182637543e-05,
        "y": 4.754653571390878e-06,
        "z": 0
      }
    },
    "name": "robot",
    "id": "be781e5ffb1e7ee4f817fe7b63e92c32",
    "robot_name": "C3PO",
    "run_id": "240"
  }
]
```

This record contains the speed, cmd_vel and position from the group "robot". Not that the tags for each measurement have been dropped. This is because they won't be used in this particular group (they can be used as a single measurement)

```json
[
  {
    "width": 384,
    "remote_paths": {
      "minio": {
        "yaml": "C3PO/2023/03/01/18/map/2023-03-01T18:20:16.yaml",
        "pgm": "C3PO/2023/03/01/18/map/2023-03-01T18:20:16.pgm"
      }
    },
    "name": "map",
    "resolution": 0.05000000074505806,
    "origin": {
      "x": -10,
      "y": -10
    },
    "local_paths": {
      "yaml": "/root/dc_data/C3PO/2023/03/01/18/map/2023-03-01T18:20:16.yaml",
      "pgm": "/root/dc_data/C3PO/2023/03/01/18/map/2023-03-01T18:20:16.pgm"
    },
    "date": 1677694816.690489,
    "height": 384,
    "id": "be781e5ffb1e7ee4f817fe7b63e92c32",
    "robot_name": "C3PO",
    "run_id": "240"
  }
]
```

This record contains the map data from the measurement.


## Configuration
### Measurement

```yaml
measurement_server:
  ros__parameters:
    custom_str_params: ["robot_name"]
    robot_name: "C3PO"
    measurement_plugins: ["cmd_vel", "map", "position", "speed"]
    save_local_base_path: "$HOME/dc_data/"
    all_base_path: "=robot_name/%Y/%m/%d/%H"
    cmd_vel:
      plugin: "dc_measurements/CmdVel"
      tags: ["flb_stdout"]
      group_key: "cmd_vel"
      enable_validator: true
      topic_output: "/dc/measurement/cmd_vel"
    position:
      plugin: "dc_measurements/Position"
      tags: ["flb_stdout"]
      group_key: "position"
      topic_output: "/dc/measurement/position"
      polling_interval: 1000
      enable_validator: true
      init_collect: true
      global_frame: "map"
      robot_base_frame: "base_link"
      transform_timeout: 0.1
    speed:
      plugin: "dc_measurements/Speed"
      tags: ["flb_stdout"]
      group_key: "speed"
      odom_topic: "/odom"
      topic_output: "/dc/measurement/speed"
    map:
      plugin: "dc_measurements/Map"
      tags: ["flb_stdout"]
      group_key: "map"
      polling_interval: 5000
      save_path: "map/%Y-%m-%dT%H:%M:%S"
      topic_output: "/dc/measurement/map"
      save_map_timeout: 4.0
      remote_prefixes: [""]
      remote_keys: ["minio"]
```

**save_local_base_path (Optional)**: Used as a common base for all saved files from measurement plugins. *all_base_path* is concatenated to it afterwards for defining the path where files are saved.

**all_base_path (Optional)**: Used as a common base for some measurements to save files. Is concatenated to *save_local_base_path*. Note the =robot_name, which is later replaced by C3PO (the variable defined in custom_str_params)

**map.remote_keys**: creates a dictionary inside **remote_paths** which is named by the strings in this field, which contains fields used by your API to fetch the remote URLs of the files.

### Group

```yaml
group_server:
  ros__parameters:
    groups: ["robot"]
    robot:
      inputs:
        [
          "/dc/measurement/cmd_vel",
          "/dc/measurement/position",
          "/dc/measurement/speed",
        ]
      output: "/dc/group/robot"
      sync_delay: 5.0
      group_key: "robot"
      tags: ["flb_stdout"]
      include_group_name: true
```

Create a group with data from cmd_vel, position and speed. Even though it appears there is nothing new here, I shall like to precise something important. In the previous demo, we mentioned that if all messages are not received by the group, it will drop it. It matters in this case because cmd_vel is not published all the time in this example (not when it is not moving), this means the data will be collected **only** when the robot moves (when a controller sends a command).

If you wished to collect the position and the speed constantly, you could take cmd_vel out of this group and add it in the destination.


#### Destinations

```yaml
destination_server:
  ros__parameters:
    flb:
      flush: 1
      flb_grace: 1
      log_level: "info"
      storage_path: "/var/log/flb-storage/"
      storage_sync: "full"
      storage_checksum: "off"
      storage_backlog_mem_limit: "1M"
      scheduler_cap: 200
      scheduler_base: 5
      http_server: true
      http_listen: "0.0.0.0"
      http_port: 2020
      in_storage_type: "filesystem"
      in_storage_pause_on_chunks_overlimit: "off"
    destination_plugins: ["flb_stdout"]
    custom_str_params_list: ["robot_name", "id"]
    custom_str_params:
      robot_name:
        name: robot_name
        value: "C3PO"
      # Requires systemd package
      id:
        name: id
        value_from_file: /etc/machine-id
    run_id:
      enabled: true
      counter: true
      counter_path: "$HOME/run_id"
      uuid: false
    flb_stdout:
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/group/robot", "/dc/measurement/map"]
      time_format: "double"
      time_key: "date"
      debug: false
```

Nothing new here, we simply edited the inputs in flb_stdout to `["/dc/group/robot", "/dc/measurement/map"]` to get the data from the robot group and the map.

## Console output

Now that the node started, let us see what's displayed in the console

```bash
[component_container_isolated-1] [INFO] [1677696067.792012776] [dc_container]: Load Library: /root/ws/install/dc_measurements/lib/libmeasurement_server_core.so
[component_container_isolated-1] [INFO] [1677696067.795921257] [dc_container]: Found class: rclcpp_components::NodeFactoryTemplate<measurement_server::MeasurementServer>
[component_container_isolated-1] [INFO] [1677696067.795956318] [dc_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<measurement_server::MeasurementServer>
[component_container_isolated-1] [INFO] [1677696067.811804437] [measurement_server]:
[component_container_isolated-1] 	measurement_server lifecycle node launched.
[component_container_isolated-1] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-1] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[component_container_isolated-1] [INFO] [1677696067.818755098] [measurement_server]: Base save path expanded to /root/dc_data/
[component_container_isolated-1] [INFO] [1677696067.818799565] [measurement_server]: All Base path expanded to C3PO/%Y/%m/%d/%H
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/measurement_server' in container 'dc_container'
[component_container_isolated-1] [INFO] [1677696067.821344112] [dc_container]: Load Library: /root/ws/install/dc_destinations/lib/libdestination_server_core.so
[component_container_isolated-1] [INFO] [1677696067.826089144] [dc_container]: Found class: rclcpp_components::NodeFactoryTemplate<destination_server::DestinationServer>
[component_container_isolated-1] [INFO] [1677696067.826138848] [dc_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<destination_server::DestinationServer>
[component_container_isolated-1] [INFO] [1677696067.845160790] [destination_server]:
[component_container_isolated-1] 	destination_server lifecycle node launched.
[component_container_isolated-1] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-1] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/destination_server' in container 'dc_container'
[component_container_isolated-1] [INFO] [1677696067.856784495] [dc_container]: Load Library: /opt/ros/humble/lib/libnav2_lifecycle_manager_core.so
[component_container_isolated-1] [INFO] [1677696067.857836219] [dc_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_lifecycle_manager::LifecycleManager>
[component_container_isolated-1] [INFO] [1677696067.857877118] [dc_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_lifecycle_manager::LifecycleManager>
[component_container_isolated-1] [INFO] [1677696067.875491851] [lifecycle_manager_navigation]: Creating
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/lifecycle_manager_navigation' in container 'dc_container'
[component_container_isolated-1] [INFO] [1677696067.881290824] [lifecycle_manager_navigation]: Creating and initializing lifecycle service clients
[component_container_isolated-1] [INFO] [1677696067.889491127] [lifecycle_manager_navigation]: Starting managed nodes bringup...
```
Measurement server and destination server are starting in the Lifecycle, you can read more about it [here](../concepts.md#lifecycle-nodes-and-bond)

"Base save path" and "All Base path" are also saved and expanded. Note "=robot_name" has been replaced by C3PO:

```bash
[component_container_isolated-1] [INFO] [1677696067.818755098] [measurement_server]: Base save path expanded to /root/dc_data/
[component_container_isolated-1] [INFO] [1677696067.818799565] [measurement_server]: All Base path expanded to C3PO/%Y/%m/%d/%H
```

Afterward, the measurement_server starts:

```
[component_container_isolated-1] [INFO] [1677694796.667666978] [lifecycle_manager_navigation]: Configuring measurement_server
[component_container_isolated-1] [INFO] [1677694796.667887325] [measurement_server]: Configuring
[component_container_isolated-1] [INFO] [1677694796.680348817] [measurement_server]: Creating measurement plugin cmd_vel: Type dc_measurements/CmdVel, Group key: cmd_vel, Polling interval: 1000, Debug: 0, Validator enabled: 1, Schema path: , Tags: [flb_stdout], Init collect: 1, Init Max measurement: 0, Include measurement name: 0, Include measurement plugin name: 0, Remote keys: , Remote prefixes: , Include measurement plugin name: 0, Max measurement on condition: 0, If all condition: , If any condition: , If none condition:
[component_container_isolated-1] [INFO] [1677694796.682517594] [measurement_server]: Configuring cmd_vel
[component_container_isolated-1] [INFO] [1677694796.684392643] [measurement_server]: Done configuring cmd_vel
[component_container_isolated-1] [INFO] [1677694796.684643767] [measurement_server]: Looking for schema at /root/ws/install/dc_measurements/share/dc_measurements/plugins/measurements/json/cmd_vel.json
[component_container_isolated-1] [INFO] [1677694796.684685350] [measurement_server]: schema: {"$defs":{"vector3":{"properties":{"x":{"description":"X speed","type":"number"},"y":{"description":"Y speed","type":"number"},"z":{"description":"Z speed","type":"number"}},"type":"object"}},"$schema":"http://json-schema.org/draft-07/schema#","description":"Command velocity sent to the robot","properties":{"angular":{"description":"Angular velocity as a vector","items":{"$ref":"#/$defs/vector3"},"type":"object"},"linear":{"description":"Linear velocity as a vector","items":{"$ref":"#/$defs/vector3"},"type":"object"}},"title":"Cmd_vel","type":"object"}
[component_container_isolated-1] [INFO] [1677694796.688143164] [measurement_server]: Creating measurement plugin map: Type dc_measurements/Map, Group key: map, Polling interval: 5000, Debug: 0, Validator enabled: 1, Schema path: , Tags: [flb_stdout], Init collect: 1, Init Max measurement: 0, Include measurement name: 1, Include measurement plugin name: 0, Remote keys: minio, Remote prefixes: , Include measurement plugin name: 0, Max measurement on condition: 0, If all condition: , If any condition: , If none condition:
[component_container_isolated-1] [INFO] [1677694796.689090088] [measurement_server]: Configuring map
[component_container_isolated-1] [INFO] [1677694796.690180661] [measurement_server]: Done configuring map
[component_container_isolated-1] [INFO] [1677694796.690522923] [measurement_server]: Looking for schema at /root/ws/install/dc_measurements/share/dc_measurements/plugins/measurements/json/map.json
[component_container_isolated-1] [INFO] [1677694796.690587274] [measurement_server]: schema: {"$defs":{"origin":{"description":"The 2-D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise rotation (yaw=0 means no rotation). Many parts of the system currently ignore yaw.","properties":{"x":{"description":"X origin of the robot","type":"number"},"y":{"description":"Y origin of the robot","type":"number"}},"type":"object"},"paths":{"properties":{"pgm":{"description":"Path to the map PGM file containing the gray-scale image","type":"string"},"yaml":{"description":"Path to the map YAML file containing map metadata","type":"string"}},"type":"object"}},"$schema":"http://json-schema.org/draft-07/schema#","description":"Map saved metadata and paths","properties":{"height":{"description":"Height of the PGM","minimum":0,"type":"integer"},"local_paths":{"description":"Paths where metadata and image are stored","items":{"$ref":"#/$defs/paths"},"type":"object"},"origin":{"description":"Robot origin position in meters","items":{"$ref":"#/$defs/origin"},"type":"object"},"remote_paths":{"additionalProperties":{"items":{"$ref":"#/$defs/paths"},"type":"object"},"description":"Dictionary of paths where metadata and image will be remotely stored","type":"object"},"resolution":{"description":"Resolution of the map, meters/pixel","minimum":0,"type":"number"},"width":{"description":"Width of the PGM","minimum":0,"type":"integer"}},"title":"Map","type":"object"}
[component_container_isolated-1] [INFO] [1677694796.693890952] [measurement_server]: Creating measurement plugin position: Type dc_measurements/Position, Group key: position, Polling interval: 1000, Debug: 0, Validator enabled: 1, Schema path: , Tags: [flb_stdout], Init collect: 1, Init Max measurement: 0, Include measurement name: 0, Include measurement plugin name: 0, Remote keys: , Remote prefixes: , Include measurement plugin name: 0, Max measurement on condition: 0, If all condition: , If any condition: , If none condition:
[component_container_isolated-1] [INFO] [1677694796.694797887] [measurement_server]: Configuring position
[component_container_isolated-1] [INFO] [1677694796.695799962] [measurement_server]: Done configuring position
[component_container_isolated-1] [INFO] [1677694796.696060904] [measurement_server]: Looking for schema at /root/ws/install/dc_measurements/share/dc_measurements/plugins/measurements/json/position.json
[component_container_isolated-1] [INFO] [1677694796.696114257] [measurement_server]: schema: {"$schema":"http://json-schema.org/draft-07/schema#","description":"Position and orientation of the robot","properties":{"x":{"description":"X position of the robot","type":"number"},"y":{"description":"Y position of the robot","type":"number"},"yaw":{"description":"Yaw angle of the robot","type":"number"}},"title":"Position","type":"object"}
[component_container_isolated-1] [INFO] [1677694796.699956883] [measurement_server]: Creating measurement plugin speed: Type dc_measurements/Speed, Group key: speed, Polling interval: 1000, Debug: 0, Validator enabled: 1, Schema path: , Tags: [flb_stdout], Init collect: 1, Init Max measurement: 0, Include measurement name: 0, Include measurement plugin name: 0, Remote keys: , Remote prefixes: , Include measurement plugin name: 0, Max measurement on condition: 0, If all condition: , If any condition: , If none condition:
[component_container_isolated-1] [INFO] [1677694796.701722429] [measurement_server]: Configuring speed
[component_container_isolated-1] [INFO] [1677694796.703057966] [measurement_server]: Done configuring speed
[component_container_isolated-1] [INFO] [1677694796.703357744] [measurement_server]: Looking for schema at /root/ws/install/dc_measurements/share/dc_measurements/plugins/measurements/json/speed.json
[component_container_isolated-1] [INFO] [1677694796.703413430] [measurement_server]: schema: {"$defs":{"vector3":{"properties":{"x":{"description":"X speed","type":"number"},"y":{"description":"Y speed","type":"number"},"z":{"description":"Z speed","type":"number"}},"type":"object"}},"$schema":"http://json-schema.org/draft-07/schema#","description":"Computed, linear and angular speed of the robot","properties":{"angular":{"description":"Angular velocity as a vector","items":{"$ref":"#/$defs/vector3"},"type":"object"},"computed":{"description":"Computed speed in meter/s","type":"number"},"linear":{"description":"Linear velocity as a vector","items":{"$ref":"#/$defs/vector3"},"type":"object"}},"title":"Speed","type":"object"}
```

Plugins are loaded one by one (here only one is, the uptime one) and configured. The configuration for each is displayed and the validation schema is also loaded and its path printed.

Then, the destination_server starts:

```
[component_container_isolated-1] [INFO] [1677694796.706019709] [lifecycle_manager_navigation]: Configuring destination_server
[component_container_isolated-1] [INFO] [1677694796.706162231] [destination_server]: Configuring
[component_container_isolated-1] [INFO] [1677694796.706907595] [destination_server]: Fluent Bit service initialized
[component_container_isolated-1] [INFO] [1677694796.707868171] [destination_server]: Creating destination plugin flb_stdout: Type dc_destinations/FlbStdout, Debug: 0, Time format: double. Time key: date
[component_container_isolated-1] [INFO] [1677694796.709554983] [destination_server]: Configuring Flb plugin flb_stdout
[component_container_isolated-1] [INFO] [1677694796.710263626] [destination_server]: Loaded lua filter. Match=ros2, code=function concatenate(tag, timestamp, record) if (type(record["tags"]) == "table") then record["tags"] = table.concat(record["tags"], ",") end  return 2, timestamp, record end
[component_container_isolated-1] [INFO] [1677694796.710309154] [destination_server]: Loaded rewrite_tag filter. Match=ros2, Rule=$tags .*(flb_stdout).* flb_stdout true
[component_container_isolated-1] [INFO] [1677694796.710382723] [destination_server]: Done configuring Flb plugin flb_stdout
[component_container_isolated-1] [INFO] [1677694796.710401937] [destination_server]: Loading input ros2 shared library /root/ws/install/fluent_bit_plugins/lib/flb-in_ros2.so...
[component_container_isolated-1] [INFO] [1677694796.711494520] [destination_server]: Loaded input ros2 shared library /root/ws/install/fluent_bit_plugins/lib/flb-in_ros2.so
[component_container_isolated-1] [INFO] [1677694796.711617719] [destination_server]: Flb ros2 plugin initialized. ret=0
[component_container_isolated-1] [INFO] [1677694796.711638189] [destination_server]: Starting Flb engine...
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [fluent bit] version=2.0.7, commit=1ab360f79c, pid=79925
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [storage] ver=1.3.0, type=memory+filesystem, sync=full, checksum=off, max_chunks_up=128
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [storage] backlog input plugin: storage_backlog.1
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [cmetrics] version=0.5.7
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [ctraces ] version=0.2.5
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [input:ros2:ros2.0] initializing
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [input:ros2:ros2.0] storage_strategy='filesystem' (memory + filesystem)
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] Started node fluentbit_rclc
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] Created subscriber /dc/group/robot
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] Created subscriber /dc/measurement/map
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [input:storage_backlog:storage_backlog.1] initializing
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [input:storage_backlog:storage_backlog.1] storage_strategy='memory' (memory only)
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [input:storage_backlog:storage_backlog.1] queue memory limit: 976.6K
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [input:emitter:emitter_for_rewrite_tag.2] initializing
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [input:emitter:emitter_for_rewrite_tag.2] storage_strategy='filesystem' (memory + filesystem)
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [output:stdout:stdout.0] worker #0 started
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [http_server] listen iface=0.0.0.0 tcp_port=2020
[component_container_isolated-1] [2023/03/01 18:19:56] [ info] [sp] stream processor started
[component_container_isolated-1] [INFO] [1677694796.763937083] [destination_server]: Started Flb engine
```

All the same as with previous demos:

It starts the [FlbStdout destination plugin](../destinations/flb_stdout.md), filters used internally by Fluent to edit Timestamps and tags. It then loads the ROS 2 Fluent Bit shared library and initialize it with the topics we provided as parameter.

Afterward, Fluent Bit starts, it prints its version, storage strategy and buffer configuration and lists which plugins are loaded.

Then, the measurement and destination server nodes are activated:

```
[component_container_isolated-1] [INFO] [1677668906.745180268] [lifecycle_manager_navigation]: Activating measurement_server
[component_container_isolated-1] [INFO] [1677668906.745462265] [measurement_server]: Activating
[component_container_isolated-1] [INFO] [1677668906.745558665] [measurement_server]: Activating uptime
[component_container_isolated-1] [INFO] [1677668906.746014639] [measurement_server]: Creating bond (measurement_server) to lifecycle manager.
[component_container_isolated-1] [INFO] [1677668906.853199899] [lifecycle_manager_navigation]: Server measurement_server connected with bond.
[component_container_isolated-1] [INFO] [1677668906.853316226] [lifecycle_manager_navigation]: Activating destination_server
[component_container_isolated-1] [INFO] [1677668906.853620571] [destination_server]: Activating
[component_container_isolated-1] [INFO] [1677668906.853722659] [destination_server]: Activating flb_stdout
[component_container_isolated-1] [INFO] [1677668906.853796078] [destination_server]: Creating bond (destination_server) to lifecycle manager.
[component_container_isolated-1] [INFO] [1677668906.961666207] [lifecycle_manager_navigation]: Server destination_server connected with bond.
[component_container_isolated-1] [INFO] [1677668906.961777773] [lifecycle_manager_navigation]: Managed nodes are active
[component_container_isolated-1] [INFO] [1677668906.961820078] [lifecycle_manager_navigation]: Creating bond timer...
```

Finally, we see the data:
```
[component_container_isolated-1] [{"remote_paths":{"minio":{"pgm":"C3PO/2023/03/01/18/map/2023-03-01T18:19:56.pgm","yaml":"C3PO/2023/03/01/18/map/2023-03-01T18:19:56.yaml"}},"date":1677694796.765904,"height":384,"name":"map","origin":{"x":-10,"y":-10},"local_paths":{"pgm":"/root/dc_data/C3PO/2023/03/01/18/map/2023-03-01T18:19:56.pgm","yaml":"/root/dc_data/C3PO/2023/03/01/18/map/2023-03-01T18:19:56.yaml"},"resolution":0.05000000074505806,"width":384,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"240"}]
[component_container_isolated-1] [{"cmd_vel":{"linear":{"x":0.246316,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.263158}},"date":1677694799.685468,"position":{"x":-0.6033772358727438,"yaw":-0.7146495585355921,"y":-1.633862970534114},"speed":{"angular":{"x":-0.0002542699063698451},"computed":3.279051750818494e-05,"linear":{"x":3.244397182637543e-05,"y":4.754653571390878e-06,"z":0}},"name":"robot","id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"240"}]
[component_container_isolated-1] [{"cmd_vel":{"linear":{"x":0.246316,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.263158}},"date":1677694800.685094,"position":{"x":-0.5307920077356227,"yaw":-0.645872441707418,"y":-1.693502983783379},"speed":{"angular":{"x":0.2127361022319145},"computed":0.2459393937023935,"linear":{"x":0.2459393844826374,"y":-6.734242603700924e-05,"z":0}},"name":"robot","id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"240"}]
[component_container_isolated-1] [{"cmd_vel":{"linear":{"x":0.26,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.368421}},"date":1677694801.685017,"position":{"x":-0.3195900881047479,"yaw":-0.3944485529461182,"y":-1.813946939716471},"speed":{"angular":{"x":0.2623361481938653},"computed":0.24584231041888,"linear":{"x":0.2458423037516516,"y":-5.725533732485466e-05,"z":0}},"name":"robot","id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"240"}]
```

So...what happened?

1. The Nav2 turtlebot3 simulation starts, a robot is able to localize and move (once you use the 2-D pose estimate on RViz)
2. The measurement plugins start publishing data to /dc/measurement/map, /dc/measurement/cmd_vel, /dc/measurement/position and /dc/measurement/speed, which contain the JSONs, tags and timestamp of the message
3. In parallel, each time the map plugin sends a ROS message, it also saves the files on the filesystem. Open a file browser to the path you set in the configuration to a path mentioned in the map JSON
4. The "robot" group node subscribes to /dc/measurement/cmd_vel, /dc/measurement/position and /dc/measurement/speed and publish on /dc/group/robot when it collects data from all 3 topics
5. Run ID and robot_name is appended in the JSON of each
6. The ROS 2 Fluent Bit plugin, which subscribes to the topics, receive the data and pass it on to Fluent Bit
7. Fluent Bit receives it, applies the timestamp filter (which modifies the timestamp to the desired format)
8. Fluent Bit applies changes the tag with another filter. This tag is used to match to the destination afterward.
9. Data is flushed
10. The Fluent Bit stdout output plugin receives the data because tags match (the flb_stdout tag is used) and forwards it to Stdout
