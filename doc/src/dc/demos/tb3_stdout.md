# Turtlebot3

In this example, we add a robot and start collecting robot data to Stdout.

You will also need 2 terminal windows, to:

1. Run the Nav2 turtlebot3 launchfile: it starts localization, navigation and RViz
2. Run DC

Since RViz is pretty verbose, using 2 terminal windows will help reading the JSON printed on the terminal window.

## Setup the environment

In each, terminal, source your environment and setup turtlebot configuration:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Nothing else has to be exported. Nav2's own `nav2_minimal_tb3_sim` ships the world, the
robot and its `ros_gz_bridge` config, and puts them on `GZ_SIM_RESOURCE_PATH` itself —
the Gazebo Classic `GAZEBO_MODEL_PATH` and `TURTLEBOT3_MODEL` variables are gone along
with Classic.

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

At the end, the data is displayed. Every Destination — `console` included — goes
through the external Vector Shipper ([ADR-0002](../adr/0002-vector-as-default-shipper.md),
[Destinations](../destinations.md)), so
each line below is Vector's own event object, one bare JSON object per line (not an
array):

```
[dc_bridge-3] {"custom_keys":["robot_name","id"],"date":1788479117.6297202,"flattened":false,"height":384,"host":"127.0.0.1","id":"e110a88ba1c24602bd2c116daf5b8287","local_paths":{"pgm":"/root/dc_data/C3PO/2026/09/03/23/map/2026-09-03T23:45:17.pgm","png":"/root/dc_data/C3PO/2026/09/03/23/map/2026-09-03T23:45:17.png","yaml":"/root/dc_data/C3PO/2026/09/03/23/map/2026-09-03T23:45:17.yaml"},"name":"map","nested":false,"origin":{"x":-10,"y":-10},"resolution":0.05000000074505806,"robot_name":"C3PO","run_id":"170","source_type":"fluent","tag":"dc.measurement.map","timestamp":"2026-09-03T23:45:17.629720410Z","width":384}
[dc_bridge-3] {"cmd_vel":{"angular":{"x":0,"y":0,"z":0.0299867},"computed":0.42023804783821106,"custom_keys":["robot_name","id"],"flattened":false,"id":"e110a88ba1c24602bd2c116daf5b8287","linear":{"x":0.420238,"y":0,"z":0},"name":"cmd_vel","nested":false,"robot_name":"C3PO","run_id":"170"},"date":1788479127.1009243,"host":"127.0.0.1","name":"robot","position":{"custom_keys":["robot_name","id"],"flattened":false,"id":"e110a88ba1c24602bd2c116daf5b8287","name":"position","nested":false,"robot_name":"C3PO","run_id":"170","x":-1.0834591164924419,"y":0.5133007443427651,"yaw":-0.008814692701341621},"source_type":"fluent","speed":{"angular":{"x":0,"y":0,"z":0.027356281873875128},"computed":0.33021257209388344,"custom_keys":["robot_name","id"],"flattened":false,"id":"e110a88ba1c24602bd2c116daf5b8287","linear":{"x":0.33021257209388344,"y":0,"z":0},"name":"speed","nested":false,"robot_name":"C3PO","run_id":"170"},"tag":"dc.group.robot","tags":[""],"timestamp":"2026-09-03T23:45:27.100924197Z"}
```

Given the JSON is quite large, let's analyze 2 different records:

The first one being the data published on the robot group. Note it's deeply nested now —
each of `cmd_vel`, `position` and `speed` carries its own copy of the custom keys
(`robot_name`, `id`, `run_id`), since those are applied per-measurement before the
group node merges them, not once at the top level:
```json
{
  "cmd_vel": {
    "angular": {
      "x": 0,
      "y": 0,
      "z": 0.0299867
    },
    "computed": 0.42023804783821106,
    "custom_keys": [
      "robot_name",
      "id"
    ],
    "flattened": false,
    "id": "e110a88ba1c24602bd2c116daf5b8287",
    "linear": {
      "x": 0.420238,
      "y": 0,
      "z": 0
    },
    "name": "cmd_vel",
    "nested": false,
    "robot_name": "C3PO",
    "run_id": "170"
  },
  "date": 1788479127.1009243,
  "host": "127.0.0.1",
  "name": "robot",
  "position": {
    "custom_keys": [
      "robot_name",
      "id"
    ],
    "flattened": false,
    "id": "e110a88ba1c24602bd2c116daf5b8287",
    "name": "position",
    "nested": false,
    "robot_name": "C3PO",
    "run_id": "170",
    "x": -1.0834591164924419,
    "y": 0.5133007443427651,
    "yaw": -0.008814692701341621
  },
  "source_type": "fluent",
  "speed": {
    "angular": {
      "x": 0,
      "y": 0,
      "z": 0.027356281873875128
    },
    "computed": 0.33021257209388344,
    "custom_keys": [
      "robot_name",
      "id"
    ],
    "flattened": false,
    "id": "e110a88ba1c24602bd2c116daf5b8287",
    "linear": {
      "x": 0.33021257209388344,
      "y": 0,
      "z": 0
    },
    "name": "speed",
    "nested": false,
    "robot_name": "C3PO",
    "run_id": "170"
  },
  "tag": "dc.group.robot",
  "tags": [
    ""
  ],
  "timestamp": "2026-09-03T23:45:27.100924197Z"
}
```

This record contains the speed, cmd_vel and position from the group "robot".

```json
{
  "custom_keys": [
    "robot_name",
    "id"
  ],
  "date": 1788479117.6297202,
  "flattened": false,
  "height": 384,
  "host": "127.0.0.1",
  "id": "e110a88ba1c24602bd2c116daf5b8287",
  "local_paths": {
    "pgm": "/root/dc_data/C3PO/2026/09/03/23/map/2026-09-03T23:45:17.pgm",
    "png": "/root/dc_data/C3PO/2026/09/03/23/map/2026-09-03T23:45:17.png",
    "yaml": "/root/dc_data/C3PO/2026/09/03/23/map/2026-09-03T23:45:17.yaml"
  },
  "name": "map",
  "nested": false,
  "origin": {
    "x": -10,
    "y": -10
  },
  "resolution": 0.05000000074505806,
  "robot_name": "C3PO",
  "run_id": "170",
  "source_type": "fluent",
  "tag": "dc.measurement.map",
  "timestamp": "2026-09-03T23:45:17.629720410Z",
  "width": 384
}
```

This record contains the map data from the measurement. There's no `remote_paths` key —
this demo's `dc_bridge` block only declares the `console` Destination below, not a
`rustfs` one, so nothing actually matches `map.remote_keys: ["rustfs"]` and the Bridge
never populates it.


## Configuration
### Measurement

```yaml
measurement_server:
  ros__parameters:
    custom_keys_str: ["robot_name"]
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

**all_base_path (Optional)**: Used as a common base for some measurements to save files. Is concatenated to *save_local_base_path*. Note the =robot_name, which is later replaced by C3PO (the variable defined in custom_keys_str)

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
    custom_key_str_list: ["robot_name", "id"]
    custom_keys_str:
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

"Base save path" and "All Base path" are also saved and expanded. Note "=robot_name" has been replaced by C3PO. `measurement_server` runs composed inside the same process as every other DC node here, so the prefix is the container's, not the node's own name:

```
[component_container_isolated-1] [INFO] [1788479023.671013064] [measurement_server]: Base save path expanded to /root/dc_data/
[component_container_isolated-1] [INFO] [1788479023.671095239] [measurement_server]: All Base path expanded to C3PO/%Y/%m/%d/%H
```

Once `dc_bridge` reports ready (per [ADR-0006](../adr/0006-bridge-outside-lifecycle-manager.md)'s `bridge_ready_gate`), the measurement plugins and the "robot" group start publishing, and we see the data on Vector's `console` sink:
```
[dc_bridge-3] {"custom_keys":["robot_name","id"],"date":1788479117.6297202,"flattened":false,"height":384,"host":"127.0.0.1","id":"e110a88ba1c24602bd2c116daf5b8287","local_paths":{"pgm":"/root/dc_data/C3PO/2026/09/03/23/map/2026-09-03T23:45:17.pgm","png":"/root/dc_data/C3PO/2026/09/03/23/map/2026-09-03T23:45:17.png","yaml":"/root/dc_data/C3PO/2026/09/03/23/map/2026-09-03T23:45:17.yaml"},"name":"map","nested":false,"origin":{"x":-10,"y":-10},"resolution":0.05000000074505806,"robot_name":"C3PO","run_id":"170","source_type":"fluent","tag":"dc.measurement.map","timestamp":"2026-09-03T23:45:17.629720410Z","width":384}
[dc_bridge-3] {"cmd_vel":{"angular":{"x":0,"y":0,"z":0.0131854},"computed":0.17549346387386322,"custom_keys":["robot_name","id"],"flattened":false,"id":"e110a88ba1c24602bd2c116daf5b8287","linear":{"x":0.175493,"y":0,"z":0},"name":"cmd_vel","nested":false,"robot_name":"C3PO","run_id":"170"},"date":1788479126.1020856,"host":"127.0.0.1","name":"robot","position":{"custom_keys":["robot_name","id"],"flattened":false,"id":"e110a88ba1c24602bd2c116daf5b8287","name":"position","nested":false,"robot_name":"C3PO","run_id":"170","x":-1.1893664880015578,"y":0.5148037648787771,"yaw":-0.02119602699838526},"source_type":"fluent","speed":{"angular":{"x":0,"y":0,"z":0},"computed":0,"custom_keys":["robot_name","id"],"flattened":false,"id":"e110a88ba1c24602bd2c116daf5b8287","linear":{"x":0,"y":0,"z":0},"name":"speed","nested":false,"robot_name":"C3PO","run_id":"170"},"tag":"dc.group.robot","tags":[""],"timestamp":"2026-09-03T23:45:26.102085677Z"}
[dc_bridge-3] {"cmd_vel":{"angular":{"x":0,"y":0,"z":0.0299867},"computed":0.42023804783821106,"custom_keys":["robot_name","id"],"flattened":false,"id":"e110a88ba1c24602bd2c116daf5b8287","linear":{"x":0.420238,"y":0,"z":0},"name":"cmd_vel","nested":false,"robot_name":"C3PO","run_id":"170"},"date":1788479127.1009243,"host":"127.0.0.1","name":"robot","position":{"custom_keys":["robot_name","id"],"flattened":false,"id":"e110a88ba1c24602bd2c116daf5b8287","name":"position","nested":false,"robot_name":"C3PO","run_id":"170","x":-1.0834591164924419,"y":0.5133007443427651,"yaw":-0.008814692701341621},"source_type":"fluent","speed":{"angular":{"x":0,"y":0,"z":0.027356281873875128},"computed":0.33021257209388344,"custom_keys":["robot_name","id"],"flattened":false,"id":"e110a88ba1c24602bd2c116daf5b8287","linear":{"x":0.33021257209388344,"y":0,"z":0},"name":"speed","nested":false,"robot_name":"C3PO","run_id":"170"},"tag":"dc.group.robot","tags":[""],"timestamp":"2026-09-03T23:45:27.100924197Z"}
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
