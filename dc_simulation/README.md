# dc_simulation

The warehouse simulation the QR-code demos run in: a gz-sim (Gazebo Harmonic) world of
QR-coded pallets, a TurtleBot3-Waffle with two RGBD cameras and a lidar, and the
`ros_gz_bridge` config that re-exposes both as ROS topics.

```bash
ros2 launch dc_simulation warehouse.launch.py            # world + robot + bridge
ros2 launch dc_simulation warehouse.launch.py headless:=True   # no GUI
```

`dc_demos`' `tb3_qrcodes.launch.py` includes this launch file and adds Nav2 and DC on
top; see [the demo page](../doc/src/dc/demos/qrcodes_minio_pgsql.md).

## Topics

`config/warehouse_bridge.yaml` maps gz-transport onto ROS. The ROS-side names are the
ones the Gazebo Classic `gazebo_ros` plugins used to publish directly, so
`dc_measurements` params, the RViz config and the docs did not have to change when the
simulation moved to gz-sim:

| ROS topic | Source |
|---|---|
| `/clock` | gz-sim |
| `/cmd_vel` (ROS→gz), `/odom`, `/tf` | `DiffDrive` system plugin |
| `/joint_states` | `JointStatePublisher` system plugin |
| `/scan` | `gpu_lidar` sensor |
| `/imu` | `imu` sensor |
| `/{left,right}_intel_realsense_r200_depth/{image_raw,camera_info,depth/image_raw}` | two `rgbd_camera` sensors |

Every sensor sets `<gz_frame_id>` to the matching link in `dc_description`'s URDF.
Without it gz-sensors stamps messages with the scoped sensor name, which is not a TF
frame, and every tf2 `MessageFilter` downstream — AMCL, both Nav2 costmaps — silently
drops the message.

## Running headless, without a GPU

The rendering sensors (`gpu_lidar` and both `rgbd_camera`s) **do** work with no GPU and
no X server, which is how CI and the dev containers run. gz-sim's `ogre2` backend falls
back to headless rendering on its own:

```
[Wrn] [Ogre2RenderEngine.cc:551] Unable to open display: . Trying to run in headless mode.
```

No extra packages, base image, or `<render_engine>` override is needed. What it costs is
speed, and the cost is large enough to plan around:

| Configuration | Real-time factor |
|---|---|
| Bare world, no robot | ~0.27 |
| Full demo: world + robot + both RGBD cameras + Nav2 | ~0.20 |
| World with the 240 pallet/bag/QR-code props removed | ~1.09 |

(measured on 8 CPUs under software rendering; a full 60-waypoint pass of the QR-code
demo took ~63 minutes of wall clock for ~8 minutes of simulated time.)

The dominant cost is the **number of model instances**, not their meshes or collision
geometry — 317 instances of 27 distinct models. Removing their collision geometry
entirely only doubles the rate; removing the instances is what buys the 4x. See
[#52](https://github.com/Minipada/ros2_data_collection/issues/52) if that matters for
your use.

One trap worth knowing, since it costs about 150x: gz-sim does **not** propagate a
wrapper `<model>`'s `<static>` into an `<include>`d nested model. `qrcodes.world` wraps
each prop in a `<static>1</static>` model, so any prop whose own `model.sdf` does not
also declare `<static>` is simulated as a free rigid body. `models/europallet`,
`models/europallet_10` and `models/bag` each declare it for exactly this reason — keep
that in mind when adding props.

Two smaller sharp edges in the same family:

- A `gpu_lidar` `<range><min>` of `0` is fatal, not merely odd: gz-rendering feeds it to
  Ogre as the ray frustum's near clip and Ogre throws
  `InvalidParametersException: Near clip distance must be greater than zero`, which
  aborts `gz sim`. Use a small positive value.
- Two `<collision>` blocks with the same name inside one `<link>` is illegal SDF. Gazebo
  Classic tolerated it; gz-sim rejects the entire spawn with
  `Error Code 2: Msg: collision with name[collision] already exists`, and the robot
  simply never appears in the world.
- A `<sensor>`'s `<pose>` is relative to its parent `<link>`, so a link pose and a sensor
  pose **add**. Writing the mounting point in both is the mistake behind
  [#51](https://github.com/Minipada/ros2_data_collection/issues/51): it put both cameras
  at twice their intended offset, and nothing complains, because a camera pointed 8
  degrees off still renders a perfectly good picture of the wrong thing.

## Camera geometry, and why the demo cares about millimetres

The QR-code demo is an inspection run: the robot stops in front of a pallet so that a
camera can read the code on it. That only works if the code lands inside the frame, and
the aisles leave very little room to be wrong in. The numbers, all measured by rendering
a code from a known pose and reading back the bounding box ZXing reports:

| Quantity | Value |
|---|---|
| Camera optical centres (from `base_footprint`) | `(0, ±0.02, 0.539)` m, looking ±Y |
| Horizontal FOV | 1.047 rad — sdformat's default, no `<horizontal_fov>` is declared |
| Frame at standoff *d* | ±*d*·tan(30°) wide, ±*d*·tan(18°) tall |
| QR symbol as rendered | 0.362 × 0.292 m, centred 0.539 m above the floor |
| Standoffs the demo drives | 0.97 m (aisle 1), 0.73 m (aisle 2), 0.98 m (aisle 3) |

Aisle 2 is the binding case: its two pallet rows are 1.49 m apart, so the robot can never
be more than ~0.73 m off either wall. A code stays readable while

```
|lateral error| + d·tan(|yaw error|) + 0.181 <= d·tan(30°)
```

which at *d* = 0.73 m leaves about 0.24 m for the goal tolerance to spend, and yaw is the
expensive half because it scales with the standoff. `dc_demos`' `qrcodes_nav.yaml`
therefore stops at 0.1 m / 0.1 rad, not the 0.2 rad it used to, and
`tools/sim/scripts/lint_launch_files.py` re-derives the whole inequality from the world,
the robot model, the waypoints and the nav params on every run, so the four cannot drift
apart again without CI saying so. The measured worst-case margin is 55 mm.
