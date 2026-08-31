# Raw topic collection

Sometimes the point is not to measure three specific things well, but to capture
*everything on the ROS graph* and put it somewhere — a bring-up session, a field
incident, a robot whose interesting topics you don't know yet.

For that, `dc_bridge` has a **generic-subscription mode**. It subscribes to topics it
was never compiled against, converts each message to JSON, and ships it to a Destination
like any other Record. No Measurement plugin, no `.msg` header, no rebuild of DC when
the robot's message packages change.

```yaml
dc_bridge:
  ros__parameters:
    shipper:
      data_dir: "$HOME/.dc/buffer"
    destinations: ["raw_console"]
    raw_console:
      type: console          # any blessed type works: postgres | s3 | file | console | vector
      receives: records
      time_key: "date"
    raw:
      enabled: true
      destination: "raw_console"
```

```bash
ros2 launch dc_bringup dc_raw.launch.py
```

`dc_raw.launch.py` starts the Bridge **alone** — no `measurement_server`, no
`group_server`, no lifecycle manager, no readiness gate (there are no collection nodes to
gate). The default params file is
[`dc_bringup/params/dc_raw_params.yaml`](https://github.com/Minipada/ros2_data_collection/blob/jazzy/dc_bringup/params/dc_raw_params.yaml);
point the launch file at your own with `dc_params_file:=…`. The same `raw:` block also
works inside a normal `dc_bringup.launch.py` params file, next to your Measurements.

## How it works

DC uses exactly what ROS 2 offers for this — the same mechanism `ros2 bag record -a` and
PlotJuggler use:

1. `get_topic_names_and_types()` lists the graph. Topics are filtered by the
   include/exclude patterns below.
2. `rclcpp::GenericSubscription` subscribes to a topic given only its *name* and *type
   string*, receiving raw serialized bytes.
3. The type's two type support libraries are loaded at run time —
   `rosidl_typesupport_cpp` to deserialize, `rosidl_typesupport_introspection_cpp` to
   describe the fields — and the message is walked into JSON.
4. The Record is Tagged `dc.raw.<topic>` and routed to `raw.destination`.

The only requirement is that the message package is on the Bridge's
`AMENT_PREFIX_PATH` — the same requirement `ros2 topic echo` has. A type that can't be
resolved is logged once and skipped; the rest of the graph is still collected.

### Tags and routing

A raw Record's Tag is the topic name with the leading `/` dropped and the rest turned
into dots, under the `dc.raw.` namespace:

| Topic | Tag | Shipper route |
|---|---|---|
| `/imu` | `dc.raw.imu` | `dc.dc.raw` |
| `/robot/battery_state` | `dc.raw.robot.battery_state` | `dc.dc.raw` |

Unlike a Measurement's Tag, the *whole namespace* shares one Vector route
(`dc.dc.raw`), matched with `starts_with` rather than an exact comparison. That is what
makes discovery possible at all: a rendered Shipper config is fixed when the Bridge
starts, but raw mode mints new Tags whenever a topic appears, and those Records still
have to reach a sink without restarting Vector. A [passthrough
sink](./destinations.md) can consume `dc.dc.raw` like any other route.

The Tag is carried on the event itself, so a single dump file or table stays
topic-attributable — check the `tag` field.

### What a Record looks like

The JSON mirrors the message structure, field for field: nested messages become nested
objects, arrays and sequences become arrays, `string` stays a string. A
`dc_interfaces/msg/StringStamped` on `/demo/custom` arrives as:

```json
{
  "tag": "dc.raw.demo.custom",
  "date": "2026-08-12T09:51:33.114777089",
  "header": {"frame_id": "base_link", "stamp": {"sec": 1786528293, "nanosec": 114777089}},
  "data": "{\"value\": 2}",
  "group_key": "demo"
}
```

Messages that start with a `std_msgs/msg/Header` are timestamped from
`header.stamp` — the moment the data was *captured*. Everything else is stamped with the
Bridge's own clock on arrival.

### Type mapping

| ROS field | JSON |
|---|---|
| `bool` | boolean |
| `int8`…`int64`, `uint8`…`uint64`, `byte`, `char` | number |
| `float32`, `float64` | number |
| `string` | string |
| `wstring` | string (transcoded UTF-16 → UTF-8) |
| a nested message | object |
| any array or sequence (fixed, bounded, unbounded) | array |

Two things about numbers are worth knowing before you point a query at the result:

- **`NaN` and `Infinity` both become `null`.** JSON has neither, so there is nowhere else
  for them to go — but it means `+Inf`, `-Inf`, `NaN` and "the producer sent nothing" are
  indistinguishable downstream. This is not a corner case: ROS messages routinely use
  `NaN` as a sentinel (`sensor_msgs/msg/BatteryState.temperature` is `NaN` when the
  battery has no temperature sensor), so expect nulls in fields whose message definition
  documents one.
- **`float32` is widened to double**, so a value written as `4.05` reads back as
  `4.050000190734863` — the exact binary32 value, not a rounding error introduced here.
  Round at the query, not in the pipeline, if the extra digits bother a dashboard.

A `uint8[]` is an array of numbers, not base64 — DC does not compress raw payloads on the
way out. That is one more reason the size and type limits below exist: the answer to a
megabyte of image bytes is to not collect it, rather than to encode it more cleverly.

## Choosing what to collect

```yaml
    raw:
      include: ["^/"]                       # topic-name regexes: the allowlist
      exclude: ["^/rosout$", "^/parameter_events$", "^/dc/measurement/", "^/dc/group/"]
      exclude_types:                        # message-type regexes
        - "^sensor_msgs/msg/(Image|CompressedImage|PointCloud|PointCloud2|LaserScan)$"
        - "^tf2_msgs/msg/TFMessage$"
      rescan_interval_secs: 5.0
```

A topic is collected when it matches at least one `include` pattern, no `exclude`
pattern, and its type matches no `exclude_types` pattern. All three are ECMAScript
regexes matched with `regex_search`, so they are **unanchored** unless you anchor them:
`camera` matches `/front/camera/info`, `^/front/` matches only that namespace.

The values above are the defaults, and they matter:

- **`/rosout` is excluded** — it carries the Bridge's own log messages, including the
  warnings raw mode emits about volume.
- **`/dc/measurement/*` and `/dc/group/*` are excluded** — those topics already reach
  their Destinations as Measurement and Group Records. Collecting them raw as well ships
  everything twice, under two different Tags.
- **High-rate sensor types are excluded**, by type rather than by name (a camera topic is
  not reliably called anything in particular). One 640×480 `sensor_msgs/msg/Image` is
  900 kB on the wire and roughly 3.7 MB once every pixel byte is a JSON number; at 30 Hz
  nothing downstream is sized for it. Override the list deliberately if you want them —
  DC will not stop you, but read the next section first.

To replace a list, write the replacement; note that an *empty* YAML list (`[]`) cannot be
loaded by rclcpp (it has no inferable element type), so use a pattern that matches
nothing, e.g. `["^$"]`, to disable a default.

`rescan_interval_secs` re-scans the graph so topics that appear after startup are picked
up — a driver started later, a node that respawned. The cost is that a topic is collected
only from ~one interval after it is first advertised: messages published before that are
not captured. Set it to `0` to scan once at startup and never again.

Topics advertising more than one type are skipped and logged: one subscription carries
one type, and picking arbitrarily would silently drop the other publisher's messages.

QoS is matched to the topic's current publishers, the same way `ros2 bag record` does
it — best effort if any publisher is best effort, transient local if all of them are.
Without that, a generic subscription silently receives nothing from a sensor driver.

## Backpressure and volume

Collecting everything can outrun the Shipper trivially. Raw mode's contract is that it
**sheds at the source rather than buffering inside the Bridge** — an unbounded in-process
queue would just move a data problem into a memory problem. Four bounds, in the order a
message meets them:

1. **`qos_depth`** (default 10) — the subscription's own history. If the Bridge is
   momentarily busy, the middleware drops the oldest messages beyond this depth before DC
   ever sees them.
2. **`max_message_size_bytes`** (default 1 MiB, 0 = unlimited) — a serialized message
   above this is dropped whole, *before* deserialization, so an unexpected point cloud
   costs nothing but a throttled warning. Treat this as a **circuit breaker, not a volume
   knob** — see the warning below.
3. **`max_rate_hz`** (default 10, per topic, 0 = unlimited) — at most one Record per
   `1/rate` seconds per topic: a message passes once that long has elapsed since the last
   one that passed, so each Record is the newest message at the moment it is emitted and
   nothing is held back. A 200 Hz topic at the default becomes 10 Hz of Records. This is
   deliberately decimation, not a token bucket: a bucket lets a burst through all at once,
   which is the exact traffic shape this exists to flatten.
4. **The Shipper refusing the Record** — if Vector is unreachable or its socket is blocked
   past the Forwarder's write timeout, the raw Record is dropped and counted. Measurement
   Records are kept in the Forwarder's unacked window for resend; raw Records are not,
   because a firehose fills that window and pushes real Records out of it.

Everything past those bounds behaves like any other Record: Vector's disk buffer
(`shipper.buffer_max_bytes`) absorbs a Destination outage, and delivery to the Shipper is
acknowledged (see [Destinations](./destinations.md#delivery-guarantees)).

### How much data is this?

These figures come off a **simulated robot**, not a spreadsheet:
`tools/sim/scripts/measure_raw_volume.sh` boots `dc_simulation`'s warehouse world, drives
the TurtleBot3-Waffle in a slow circle, points a Bridge in raw mode at its live topics,
and reports what a `file` Destination stored, per Tag. Everything below is one run of it —
re-run it after anything that changes what a Record costs. It is a local tool, not a CI
gate; its header explains the three configurations and why its rates are counted in
*simulated* seconds.

Rates are that world's Waffle's: IMU 200 Hz, odometry and TF 30 Hz (the `DiffDrive`
plugin), lidar and both RGBD cameras 5 Hz. `/cmd_vel` and `/clock` are left out — the
first is the benchmark's own driving, the second exists only because the robot is
simulated.

| Configuration | Shipped |
|---|---|
| **Defaults** (10 Hz cap, sensor types excluded) | 24 kB/s — **86 MB/hour, 2.1 GB/day** |
| …plus `scan` and `tf` re-enabled | 69 kB/s — **5.9 GB/day** |
| …plus both 1280×720 cameras re-enabled | ≈ **9.5 TB/day**, if anything could carry it |

Per-Record cost, which is what to multiply by your own topics' rates:

| Topic (message) | Publishes at | Bytes per Record |
|---|---|---|
| `/joint_states` (`JointState`, 2 joints) | 1 000 Hz † | 355 |
| `/tf` (`TFMessage`, 1 transform) | 30 Hz | 406 |
| `…/camera_info` (`CameraInfo`) | 5 Hz | 581 |
| `/odom` (`Odometry`) | 30 Hz | 669 |
| `/imu` (`Imu`) | 200 Hz | 767 |
| `/scan` (`LaserScan`, 360 ranges + intensities) | 5 Hz | 7 878 |
| `…/image_raw` (`Image`, 1280×720 `rgb8`) | 5 Hz | **10 981 032** |

† the simulator's `JointStatePublisher` runs every physics step; a real driver is far
slower. It makes no difference to the total, which is the point of the next paragraph.

**The rate cap is what makes the first row affordable, not the topic list.** A topic
publishing faster than `max_rate_hz` contributes `bytes_per_record × 10` per second no
matter how fast it actually runs, so `joint_states` at 1 000 Hz and `imu` at 200 Hz cost
3.6 kB/s and 7.7 kB/s respectively. Only the topics *below* the cap — the 5 Hz sensors —
bill at their real rate, which is why re-enabling one 5 Hz lidar (7.9 kB per Record, 40
kB/s, 3.5 GB/day) nearly triples the total on its own.

The last row is the whole argument for the default type exclusions: one 1280×720 frame
costs as much as ~16 000 odometry Records. JSON does not merely double a byte array —
`10 981 032` bytes for a 2 764 800-byte frame is **four times** the wire size, because
most pixel values print as three digits and a comma.

Two cameras at 5 Hz is 110 MB/s of Records, and **nothing in the pipeline carries that**,
which is why that row says "if anything could carry it": in the run it comes from, the
Shipper refused 475 of the 894 Records offered to it (`dropped … 475 shipper`, 53 %). The
collapse was not confined to the images either — `imu` arrived at 4 Hz instead of 200 and
`odom` at 0.7 Hz instead of 30, because the Bridge spent the window serializing frames.
Collecting a camera raw does not cost you a camera's worth of storage; it costs you the
rest of your collection.

**The size cap will not save you from a camera** — and it should not be asked to. A
640×480 `rgb8` frame is 921 600 bytes on the wire, *under* the default
`max_message_size_bytes` of 1 MiB, so it passes the size gate and lands as megabytes of
JSON. (This world's 1280×720 frames are three times that and the default cap does drop
them whole — which is why the benchmark's camera profile has to lift it to measure
anything at all. Relying on that is a collection policy that silently switches on the day
someone fits a smaller sensor.) It is `exclude_types` that keeps images out, and removing
that list removes the protection entirely.

The tempting fix — lower the size cap until frames stop fitting — is a bad trade, because
**the two limits fail differently**:

- `max_rate_hz` drops *by time*, uniformly. Every window is still represented, so you get
  an honest lower-resolution time series.
- `max_message_size_bytes` drops *by content*. On any topic whose messages vary in size —
  a compressed image, a point cloud that grows with scene complexity, a diagnostics array,
  a string payload — it removes precisely the large ones and keeps the small ones. The
  result looks complete and is silently biased toward the least interesting data, with
  nothing in the stored Records to say what went missing.

So set the size cap high enough that it essentially never fires, and treat it as a circuit
breaker against the pathological and unanticipated (the topic you did not know carried
100 MB). To *not* collect something, exclude it by type or topic: all-or-nothing, visible
in the startup log, and no biased sample. The only case for a low size cap is when you
genuinely want "this topic, except its outliers" — which is rarely what anyone means.

Every drop is counted, and the counters are on the Bridge's readiness service — the
fastest way to answer "is raw mode shedding?":

```console
$ ros2 service call /dc_bridge/ready std_srvs/srv/Trigger
response: success=True, message='vector is accepting connections | raw: 12 topic(s),
  3480 forwarded, dropped 51200 rate / 0 oversize / 0 shipper / 0 undecodable'
```

`dropped rate` climbing is normal and healthy (it is the limiter doing its job).
`dropped shipper` climbing means the pipeline is genuinely behind: raise
`shipper.buffer_max_bytes`, lower `max_rate_hz`, or narrow `include`.

## When *not* to use it

Raw mode ships whatever a topic happens to contain. It has no
[Conditions](./conditions.md), no [data validation](./data_validation.md), no
[Groups](./groups.md), and no File uploads — a Measurement is still the right tool for
data you know you want, in a shape you control, joined with other data. Raw mode is for
the case where you don't know yet, or where "all of it" *is* the requirement.

It is also not a rosbag replacement: JSON in Postgres is not a replayable recording. If
you want replay, point a [passthrough sink](./demos/mcap_recording.md) at the
`dc.dc.raw` route and let `dc_mcap_writer` record it, or run `ros2 bag record` alongside
DC.
