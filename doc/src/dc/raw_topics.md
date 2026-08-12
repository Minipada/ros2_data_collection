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
      type: console          # any blessed type works: postgres | s3 | file | console
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
  roughly 900 kB of JSON numbers; at 30 Hz nothing downstream is sized for it. Override
  the list deliberately if you want them — DC will not stop you, but read the next
  section first.

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
   costs nothing but a throttled warning.
3. **`max_rate_hz`** (default 10, per topic, 0 = unlimited) — at most one Record per
   `1/rate` seconds per topic, newest wins. A 200 Hz topic at the default becomes 10 Hz of
   Records. This is deliberately decimation, not a token bucket: a bucket lets a burst
   through all at once, which is the exact traffic shape this exists to flatten.
4. **The Shipper refusing the Record** — if Vector is unreachable or its socket is blocked
   past the Forwarder's write timeout, the raw Record is dropped and counted. Measurement
   Records are kept in the Forwarder's unacked window for resend; raw Records are not,
   because a firehose fills that window and pushes real Records out of it.

Everything past those bounds behaves like any other Record: Vector's disk buffer
(`shipper.buffer_max_bytes`) absorbs a Destination outage, and delivery to the Shipper is
acknowledged (see [Destinations](./destinations.md#delivery-guarantees)).

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
