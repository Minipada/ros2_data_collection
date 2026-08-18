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
| `/battery_state` | `LinearBatteryPlugin` (`worlds/waffle.model`) — see "Battery" below |

Every sensor sets `<gz_frame_id>` to the matching link in `dc_description`'s URDF.
Without it gz-sensors stamps messages with the scoped sensor name, which is not a TF
frame, and every tf2 `MessageFilter` downstream — AMCL, both Nav2 costmaps — silently
drops the message.

## Battery

`worlds/waffle.model` carries a `gz-sim-linearbatteryplugin-system` battery
(`libgz-sim8-linearbatteryplugin-system.so`, already in the workspace image). It
discharges once the robot has driven at all — the plugin's own rule, not a `dc_simulation`
choice: current draw latches on at the first nonzero wheel command and never resets
itself — and recharges when it is told to over `gz.msgs.Boolean` services, which
`models/charging_dock/model.sdf` turns into "when the robot noses up to a pad in the
world" using two stock gz-sim systems (`TouchPlugin`, `TriggeredPublisher`) and no new
code. `ros_gz_bridge` re-publishes the plugin's own `.../battery/linear_battery/state`
topic as ROS `battery_state`, the unqualified name `opennav_docking`'s
`SimpleChargingDock` subscribes to (`dc_demos/params/qrcodes_nav.yaml`'s
`docking_server.use_battery_status`, on since #364).

**Rates, and why.** The real TurtleBot3-Waffle spec — an 11.1 V Li-ion pack, about
1.8 Ah — is kept where it costs nothing (`voltage`), but not for `capacity`: at the real
size, a visible discharge needs on the order of two hours of continuous driving. Rates
matter more than realism here, so `capacity` is 0.05 Ah, sized against `power_load` and
`charging_time` so one discharge-then-recharge cycle fits in a couple of minutes:

- `power_load` 18.0 W draws about 1.62 A at 11.1 V (`I = P / V`). Emptying 0.05 Ah at
  1.62 A takes `capacity / I` ≈ 111 s of driving.
- `charging_time` 0.02 h (72 s) is the SDF parameter's own unit — hours to go from empty
  to full at the derived charging current (`capacity / charging_time`, 2.5 A here, above
  the 1.62 A discharge current as gz-sim's own `linear_battery_demo.sdf` recommends).
  `LinearBatteryPlugin` stops adding that current past 90% state of charge, so reaching
  the cap from empty takes about `0.9 * 72 s` ≈ 65 s; `charging_dock/model.sdf`'s
  `TriggeredPublisher` waits 90 s before calling `recharge/stop`, comfortably past that.

**Docking mechanism.** `models/charging_dock/model.sdf` places a contact-sensor pad 0.8 m
ahead of `warehouse.launch.py`'s default spawn pose. A `TouchPlugin` fires once when a
`turtlebot3_waffle` collision has touched the pad continuously for 1 s, and two
`TriggeredPublisher` plugins turn that into a `recharge/start` call and, after the 90 s
above, a `recharge/stop` call plus re-arming the `TouchPlugin` for the next visit. Try it
by hand against `warehouse.launch.py` (headless or not):

```bash
gz topic -e -t /model/turtlebot3_waffle/battery/linear_battery/state   # watch it discharge
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.3}"            # drive to the dock
```

**Sign convention.** `sensor_msgs/BatteryState` documents positive `current` as charging,
negative as discharging; gz's own internal sign is the opposite (a plain discharge
current is positive, and charging current is *subtracted*, so it only ever pushes the
raw value negative). `LinearBatteryPlugin`'s `invert_current_sign` flips it, because
`SimpleChargingDock` decides `is_charging_` from `current > charging_threshold_` (0.5 A
default) — without the inversion it would never see a charge.

**Percentage scale.** `fix_issue_225` reports `percentage` on a 0–100 scale rather than
gz's un-fixed 0–1. That matches `dc_measurements`' battery plugin's own
`percentage_scale` default of 100.0 (`plugins/measurements/json/battery.json`, #361) —
built for exactly this gz-sim quirk, since `ros_gz_bridge` copies the field straight
through with no rescaling of its own.

## What the robot's topics cost to collect

```bash
./tools/sim/scripts/measure_raw_volume.sh     # per-Tag bytes, rates and a GB/day
```

Boots this world headless, drives the robot, runs `dc_bridge` in raw mode against its
topics and reports what each one is worth in stored bytes — the source of the figures in
[Raw topic collection](../doc/src/dc/raw_topics.md#how-much-data-is-this). Local and
informational, never a CI gate; see the script's header for the profiles and why its rates
are counted in simulated seconds.

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
| World + robot + both RGBD cameras, no Nav2, no DC | ~0.11 – 0.14 |
| Full demo: world + robot + both RGBD cameras + Nav2, DC off | ~0.20 |
| Full demo, DC running (camera measurements decoding every poll) | ~0.11 |
| World with the 240 pallet/bag/QR-code props removed entirely | ~1.09 |

(measured on 8 CPUs under software rendering; a full 60-waypoint pass of the QR-code
demo took ~63 minutes of wall clock for ~8 minutes of simulated time with DC off.)

The dominant cost with Nav2 and DC off is the **number of model instances**, not their
meshes or collision geometry: `qrcodes.world` declared 318 top-level `<model>` entities
of 27 distinct models (`grep -c '<model name=' dc_simulation/worlds/qrcodes.world`).
Removing their collision geometry entirely only doubles the rate; removing the instances
is what buys the 4x.

#52 acted on that finding where it could be acted on for free: every QR-coded station
wrapped its pallet (`europallet`) and the bag sitting on it (`bag`) in two separate
top-level `<model>` entities at the same pose, purely because the world file was
generated that way — nothing needs them to be separate gz-sim entities. Merging the pair
into one top-level `<model>` with two sibling `<include>`s (see `europallet_bag_1_1` for
an example) takes the file from 318 to 238 top-level instances with **zero** change to
any collision geometry, visual mesh, or absolute pose — `tools/sim/scripts/
lint_launch_files.py`'s QR-alignment check (#51) passes unchanged before and after,
since every `qrcode_*` model is still its own top-level entity at its original pose. That
alone measured ~0.11 → ~0.18 RTF for "world + robot + both RGBD cameras, no Nav2, no DC"
— the QR-code models themselves (a further 80 entities) were deliberately left
unmerged, because `lint_launch_files.py`'s `read_code_planes()` currently reads a QR
code's absolute pose straight off its wrapping `<model>`'s own `<pose>`, and merging
would require it to compose that with a nested `<include>`'s relative pose instead. That
is a real further win — worth a follow-up — but changing the one check that exists
specifically to catch #51-style regressions felt like the wrong thing to bundle into a
world-file cleanup.

**That gain does not show up once Nav2 and DC are both running.** With the full demo
pipeline up and the robot merely parked at its spawn pose — no navigation, camera
measurements still polling once a second — RTF sits at ~0.11 whether or not the props
are merged: the same figure progress.txt records for "the pass with DC running" against
#279's DC-off 0.198. Nav2's planners/costmaps and DC's per-frame ZXing decode (two
1280x720 streams, every poll) are together the larger cost once they're both in the
loop, and they swamp what the world-entity count buys on this box. See
[#52](https://github.com/Minipada/ros2_data_collection/issues/52) for the full
measurement writeup, including time-to-first-Record and where that time actually goes.

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
| Horizontal FOV | 1.047 rad (60°) — declared, see below |
| Frame at standoff *d* | ±*d*·tan(30°) wide, ±*d*·tan(18°) tall |
| QR symbol as rendered | 0.362 × 0.292 m, centred 0.539 m above the floor |
| Standoffs the demo drives | 0.97 m (aisle 1), 0.73 m (aisle 2), 0.98 m (aisle 3) |

Aisle 2 is the binding case: its two pallet rows are 1.49 m apart, so the robot can never
be more than ~0.73 m off either wall. A code stays readable while

```
|lateral error| + d·tan(|yaw error|) + 0.181 <= d·tan(30°)
```

`tools/sim/scripts/lint_launch_files.py` re-derives that inequality — and its vertical
twin — from the world, the robot model, the waypoints and the nav params on every run, so
the four cannot drift apart again without CI saying so. The worst case over the goal
tolerance is **55 mm**.

### Why the field of view is 60°, and why widening it did not work

Neither camera used to declare `<horizontal_fov>`, so both inherited sdformat's fallback
of 1.047 rad. Both declare it now. The value is unchanged, so nothing renders
differently; what changed is that the number is visible and the lint reads it rather than
assuming it — the check and the simulator can no longer disagree about the one number the
whole inequality turns on.

60° is narrower than the real cameras this robot is named for. The SDF sensors are named
for an **R200** (~77° colour, ~70° depth); `dc_description` instantiates a **D455**
(~90° colour, ~87° depth) via `realsense2_description`. Nothing upstream helps: that
package is URDF and meshes only, with no gz sensor definition and no `horizontal_fov`
anywhere, for any model. The only gz camera sensors in the ROS install are nav2's own
`gz_waffle` (1.047) and the TB4's OAK-D (1.25).

Widening to the R200's 77° was tried, because 55 mm of margin is thinner than it looks —
the goal checker compares AMCL's *estimate* against the goal, so localization error lands
on top of the tolerance and never appears in that budget. It takes the worst case from
55 mm to 134 mm. **It was reverted anyway**: at 77° the code at aisle 1 station 9 stops
decoding at a pose Nav2 is allowed to stop in, one the 60° lens reads without trouble.
That is not a resolution effect — the same view rendered at 1920×1080 also fails, while
simply upscaling the 1280×720 frame 1.5× decodes it — so something about the wider view
defeats ZXing's detector rather than starving it of pixels. Until that is understood, the
lens that is measured good at every station wins over the one that is more realistic.

## Lighting: verified stable on gz-sim, #61

[#61](https://github.com/Minipada/ros2_data_collection/issues/61) was filed against
Gazebo Classic (OGRE 1.x): the world rendered too dark intermittently, which broke QR
detection. #268 replaced the renderer wholesale (OGRE 1.x → gz-sim/gz-rendering OGRE2),
so this was re-scoped as a verification task rather than a known bug — either the old
symptom is gone under the new renderer, or it presents differently and needs its own fix.

**It does not reproduce.** Five independent cold starts of the full `tb3_qrcodes.launch.py`
pipeline — two on GitHub-hosted CI runners (commits `41e24d0`, 6/6 stations decoded, and
`2b03c78`, 5/6), three run locally against the same CI-built image (2/3, 2/3, and 3/4,
the last against the world file this section documents) — each read a QR code at every
station bar the one-station gap `tools/sim/scripts/run.sh`'s own `detect` stage already
treats as expected (the goal-reached and camera-poll events race by design; see the
script's own comment). No run produced a degenerate (all-black or otherwise all-uniform)
camera frame.

That is consistent with the world having no source of *intermittent* lighting to begin
with: `qrcodes.world`'s `<scene>` carries a fixed `<ambient>`/`<background>` (now declared
explicitly rather than left to sdformat's fallback — the same reasoning already applied to
`<horizontal_fov>` above), a single `directional` sun at a fixed `<direction>`, no sky/
time-of-day element and no light-flicker plugin. Nothing in the file varies frame to frame
or run to run. Gazebo Classic's OGRE1 backend is a different renderer with its own
initialization quirks; whatever produced the original symptom does not carry over.

**Closed with this evidence** rather than left open per the issue's own acceptance
criteria — reopen if a dark frame turns up again, with the run's log attached.
