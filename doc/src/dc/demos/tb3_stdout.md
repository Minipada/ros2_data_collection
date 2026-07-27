# Turtlebot3

In this example, we add a robot and start collecting robot data to Stdout.

You will also need 2 terminal windows, to:

1. Run the Nav2 turtlebot3 launchfile: it starts localization, navigation and RViz
2. Run DC

Since RViz is pretty verbose, using 2 terminal windows will help reading the JSON printed on the terminal window.

## Setup the environment

In each, terminal, source your environment and setup turtlebot configuration:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=waffle
```

## Start Navigation

Then, start the Turtlebot launchfile:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

RViz and Gazebo will start: you should now see the robot in Gazebo, and the map on RViz.

Set the robot position using the "2D Pose Estimate" button.

```admonish info
If any problem occur, please take a look at the [nav2 official documentation](https://navigation.ros.org/getting_started/index.html#running-the-example) which covers the case.
```

## Start DC

Execute

```bash
ros2 launch dc_demos tb3_simulation_stdout.launch.py
```

At the end, the data is displayed:

```
{"date":1677690777.96911,"width":384,"height":384,"remote_paths":{"rustfs":{"pgm":"C3PO/2023/03/01/17/map/2023-03-01T17:12:57.pgm","yaml":"C3PO/2023/03/01/17/map/2023-03-01T17:12:57.yaml"}},"resolution":0.05000000074505806,"origin":{"x":-10,"y":-10},"local_paths":{"pgm":"/root/dc_data/C3PO/2023/03/01/17/map/2023-03-01T17:12:57.pgm","yaml":"/root/dc_data/C3PO/2023/03/01/17/map/2023-03-01T17:12:57.yaml"},"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}
{"position":{"x":-0.7254206029057992,"yaw":0.134098723062189,"y":-0.5116378019627142},"cmd_vel":{"linear":{"x":0.26,"z":0,"y":0},"angular":{"x":0,"z":-0.157895,"y":0}},"speed":{"linear":{"x":8.720555295508514e-05,"z":0,"y":4.219923511090644e-06},"computed":8.730759543499989e-05,"angular":{"x":-0.0002958005596118955}},"date":1677690787.878526,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}
{"position":{"x":-0.1931822640325796,"yaw":-0.08553498481283986,"y":-0.5267276846092974},"cmd_vel":{"linear":{"x":0.232632,"z":0,"y":0},"angular":{"x":0,"z":-0.0526316,"y":0}},"speed":{"linear":{"x":0.2466804589428482,"z":0,"y":4.147463233418404e-05},"computed":0.2466804624294339,"angular":{"x":-0.1575310923819759}},"date":1677690789.878385,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"234"}
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

This record contains the speed, cmd_vel and position from the group "robot".

```json
[
  {
    "width": 384,
    "remote_paths": {
      "rustfs": {
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
    run_id:
      enabled: true
      counter: true
      counter_path: "$HOME/run_id"
      uuid: false
    save_local_base_path: "$HOME/dc_data/"
    all_base_path: "=robot_name/%Y/%m/%d/%H"
    cmd_vel:
      plugin: "dc_measurements/CmdVel"
      group_key: "cmd_vel"
      enable_validator: true
      topic_output: "/dc/measurement/cmd_vel"
    position:
      plugin: "dc_measurements/Position"
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
      group_key: "speed"
      odom_topic: "/odom"
      topic_output: "/dc/measurement/speed"
    map:
      plugin: "dc_measurements/Map"
      group_key: "map"
      polling_interval: 5000
      save_path: "map/%Y-%m-%dT%H:%M:%S"
      topic_output: "/dc/measurement/map"
      save_map_timeout: 4.0
      remote_prefixes: [""]
      remote_keys: ["rustfs"]
```

**save_local_base_path (Optional)**: Used as a common base for all saved files from measurement plugins. *all_base_path* is concatenated to it afterwards for defining the path where files are saved.

**all_base_path (Optional)**: Used as a common base for some measurements to save files. Is concatenated to *save_local_base_path*. Note the =robot_name, which is later replaced by C3PO (the variable defined in custom_str_params)

**map.remote_keys**: creates a dictionary inside **remote_paths** which is named by the strings in this field — each name must match a `receives: files` Destination in the `dc_bridge` block below, so the Bridge's Uploader knows where to send the file.

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
      include_group_name: true
```

Create a group with data from cmd_vel, position and speed. Even though it appears there is nothing new here, I shall like to precise something important. In the previous demo, we mentioned that if all messages are not received by the group, it will drop it. It matters in this case because cmd_vel is not published all the time in this example (not when it is not moving), this means the data will be collected **only** when the robot moves (when a controller sends a command).

If you wished to collect the position and the speed constantly, you could take cmd_vel out of this group and add it in the destination.


#### Destinations

```yaml
dc_bridge:
  ros__parameters:
    shipper:
      data_dir: "$HOME/.dc/buffer"
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/group/robot", "/dc/measurement/map"]
      time_key: "date"
      time_format: "double"
    vector_forward_host: "127.0.0.1"
    vector_forward_port: 24224

measurement_server:
  ros__parameters:
    custom_str_params_list: ["robot_name", "id"]
    custom_str_params:
      robot_name:
        name: robot_name
        value: "C3PO"
      # Requires systemd package
      id:
        name: id
        value_from_file: /etc/machine-id
```

Nothing new here, we simply edited the `console` Destination's `inputs` to `["/dc/group/robot", "/dc/measurement/map"]` to get the data from the robot group and the map.

## Console output

Now that the node started, let us see what's displayed in the console. Measurement server and `dc_bridge` are started in the Lifecycle, you can read more about it [here](../concepts.md#lifecycle-nodes-and-bond).

"Base save path" and "All Base path" are also saved and expanded. Note "=robot_name" has been replaced by C3PO:

```
[measurement_server-1] [INFO] [measurement_server]: Base save path expanded to /root/dc_data/
[measurement_server-1] [INFO] [measurement_server]: All Base path expanded to C3PO/%Y/%m/%d/%H
```

Once `dc_bridge` reports ready (per [ADR-0006](https://github.com/Minipada/ros2_data_collection/blob/jazzy/docs/adr/0006-bridge-outside-lifecycle-manager.md)'s `bridge_ready_gate`), the measurement plugins and the "robot" group start publishing, and we see the data on Vector's `console` sink:
```
{"remote_paths":{"rustfs":{"pgm":"C3PO/2023/03/01/18/map/2023-03-01T18:19:56.pgm","yaml":"C3PO/2023/03/01/18/map/2023-03-01T18:19:56.yaml"}},"date":1677694796.765904,"height":384,"name":"map","origin":{"x":-10,"y":-10},"local_paths":{"pgm":"/root/dc_data/C3PO/2023/03/01/18/map/2023-03-01T18:19:56.pgm","yaml":"/root/dc_data/C3PO/2023/03/01/18/map/2023-03-01T18:19:56.yaml"},"resolution":0.05000000074505806,"width":384,"id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"240"}
{"cmd_vel":{"linear":{"x":0.246316,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.263158}},"date":1677694799.685468,"position":{"x":-0.6033772358727438,"yaw":-0.7146495585355921,"y":-1.633862970534114},"speed":{"angular":{"x":-0.0002542699063698451},"computed":3.279051750818494e-05,"linear":{"x":3.244397182637543e-05,"y":4.754653571390878e-06,"z":0}},"name":"robot","id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"240"}
{"cmd_vel":{"linear":{"x":0.246316,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.263158}},"date":1677694800.685094,"position":{"x":-0.5307920077356227,"yaw":-0.645872441707418,"y":-1.693502983783379},"speed":{"angular":{"x":0.2127361022319145},"computed":0.2459393937023935,"linear":{"x":0.2459393844826374,"y":-6.734242603700924e-05,"z":0}},"name":"robot","id":"be781e5ffb1e7ee4f817fe7b63e92c32","robot_name":"C3PO","run_id":"240"}
```

So...what happened?

1. The Nav2 turtlebot3 simulation starts, a robot is able to localize and move (once you use the 2-D pose estimate on RViz)
2. The measurement plugins start publishing data to /dc/measurement/map, /dc/measurement/cmd_vel, /dc/measurement/position and /dc/measurement/speed, which contain the JSON and timestamp of the message
3. In parallel, each time the map plugin sends a ROS message, it also saves the files on the filesystem. Open a file browser to the path you set in the configuration to a path mentioned in the map JSON
4. The "robot" group node subscribes to /dc/measurement/cmd_vel, /dc/measurement/position and /dc/measurement/speed and publish on /dc/group/robot when it collects data from all 3 topics
5. Run ID and robot_name is appended in the JSON of each
6. `dc_bridge`, which subscribes to `/dc/group/robot` and `/dc/measurement/map` directly, receives the data and forwards it to the external Vector process over the shipper ingest protocol
7. Vector's generated config applies a `remap` transform that writes the configured `time_key` in the requested `time_format`
8. Vector's `console` sink, the only one matching the `console` Destination we configured, prints the JSON to stdout
