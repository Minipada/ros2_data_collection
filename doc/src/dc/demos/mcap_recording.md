# MCAP recording (passthrough)

`dc_bridge` blesses exactly five Destination types — `postgres`, `s3`, `file`, `console`,
`vector` (see [Destinations](../destinations.md)) — and MCAP is not one of them; Vector, the
Shipper, has no MCAP sink at all. This tutorial is the worked example for #210: the
ADR-0003 **passthrough** plus a small standalone process, `dc_mcap_writer`
(ADR-0009, [`docs/adr/`](https://github.com/minipada/ros2_data_collection/tree/jazzy/docs/adr)),
that consumes the same public `dc.<tag>` routes a blessed Destination consumes and
writes them as rotated `.mcap` files, ready to open with `ros2 bag info` or
[Foxglove](https://foxglove.dev/).

Unlike the [Elasticsearch](./elasticsearch.md)/[InfluxDB](./tb3_aws_influxdb.md)
passthrough demos, there is no raw Vector TOML to hand-author and no second terminal to
start a companion process in: `dc_bringup.launch.py` reads a `dc_mcap_writer:` block
from the same params file every other node's parameters live in, generates the
passthrough sink from it, and starts `dc_mcap_writer` itself automatically. From the
params file it *looks* like configuring a Destination — one block, `inputs`, done —
even though it structurally isn't one (see "Understanding the configuration" below for
why).

```admonish info
Each Record is written as a JSON-schema-encoded MCAP message — one Channel per Tag,
schema `{"type": "object"}` — the pattern in the
[`foxglove/mcap` `jsonschema/writer.cpp`](https://github.com/foxglove/mcap/blob/main/cpp/examples/jsonschema/writer.cpp)
example issue #210 links to. This is not a `ros2 bag record` capture of typed ROS
messages: Records are DC's own JSON payloads, not (de)serialized ROS message types, so
there is nothing to generate `.msg`/`.idl` schemas from. `ros2 bag info` reads the
file's Channel/Statistics records regardless of encoding; Foxglove additionally
understands `jsonschema` channels well enough to plot and inspect fields directly.
```

## Run it

Hardware-free: four system Measurements, one `dc_mcap_writer:` block in the params
file (`dc_demos/params/mcap_recording.yaml`), one launch command:

```bash
colcon build
ros2 launch dc_demos mcap_recording.launch.py
```

`dc_bringup.launch.py` starts `dc_mcap_writer` before the rest of the stack, so its TCP
listener is up before Vector's generated `socket` sink (re)connects to it — the very
first Record lands instead of relying on Vector's own retry. The `console` Destination
prints every Record as it is shipped, so the terminal doubles as a local view of what
`dc_mcap_writer` is receiving; `dc_mcap_writer`'s own log lines (also on this terminal —
it runs alongside `dc_bridge`, not in the background) show each file it opens and closes.

## Verify the recording

Stop the stack (`Ctrl-C`) once a few Records have been collected — `dc_mcap_writer`
finishes the file it has open on shutdown, so every `.mcap` under the configured
`output_dir` (`~/dc_mcap_out` in the demo params) is independently valid, not just the
most recent rotation:

```bash
ros2 bag info ~/dc_mcap_out/records_<timestamp>_<pid>_0001.mcap
```

```
Files:             records_20260811T090626Z_4213_0001.mcap
Bag size:          .. KiB
Storage id:        mcap
Duration:          ...s
Start:             ...
End:               ...
Messages:          20
Topic information: Topic: dc.dc.measurement.cpu    | Type: unknown | Count: 5 | ...
                    Topic: dc.dc.measurement.memory | Type: unknown | Count: 5 | ...
                    Topic: dc.dc.measurement.os      | Type: unknown | Count: 1 | ...
                    Topic: dc.dc.measurement.uptime  | Type: unknown | Count: 9 | ...
```

`Type: unknown` is expected — these Channels are JSON-schema, not a registered ROS
message type, so rosbag2 has nothing to resolve the type name to. Message counts and
per-topic breakdown come from the file's Statistics/Channel records regardless.

Opening the same file in [Foxglove Studio](https://foxglove.dev/) (**Open local
file…**) lists each `dc.<tag>` Channel and renders its JSON fields (`cpu.average`,
`memory.used`, `uptime.time`, …) in the Raw Messages and Plot panels like any other
topic.

## Rotation

`dc_mcap_writer` rotates to a new `.mcap` file once the current one hits `max_bytes`
(default 128 MiB) or has been open `max_duration_secs` (default 300s), whichever comes
first — the same "whichever limit first" shape used for `files.retention` (ADR-0005),
applied here to `dc_mcap_writer`'s own output rather than the Bridge's Uploader queue
(ADR-0009 explains why this stays outside `dc_bridge`). Filenames are
`<prefix>_<UTC timestamp>_<pid>_<rotation index>.mcap`; the PID and counter together
guarantee a unique name even across a process restart landing in the same wall-clock
second as the previous process's last rotation — without both, the new process could
compute the identical name and its `open(..., "wb")` would silently truncate the file
the previous process had already finished.

```admonish warning
A `.mcap` file only becomes readable once its rotation finishes — writing the file's
closing footer is what `max_bytes`/`max_duration_secs` triggers, not something every
individual Record write does. A process that never gets to shut down gracefully
(`SIGKILL`, or a container/orchestrator grace period too short for the clean-shutdown
path to complete) loses whatever is in the file still open at that moment; every
already-finished rotation stays valid and readable regardless. `max_duration_secs`'s
default balances this against not producing too many small files — lower it for
tighter durability, raise it for fewer files, per your own tolerance for that
loss window.
```

## Understanding the configuration

```admonish info
See the [Elasticsearch tutorial](./elasticsearch.md) for the full ADR-0003 passthrough
mechanics — routing, buffering, the `dc.<tag>` contract. This section only covers
what's specific to how MCAP recording is wired up.
```

`dc_demos/params/mcap_recording.yaml`'s `dc_mcap_writer:` block is **not** nested
inside `dc_bridge`'s `destinations` list, and can't be made to look exactly like
`postgres`/`s3`/`file`/`console`/`vector` there: `destinations` is parsed and validated by
`dc_bridge` itself, in C++, and an unrecognized `type` is a hard startup error by
design (see [Destinations](../destinations.md)) — teaching it a sixth type would mean
changing `dc_bridge`, which ADR-0009 explicitly decided against (Vector's own
at-least-once/disk-buffered guarantees already cover what would have justified that).
So it's a sibling top-level block instead, structurally shaped like a Destination
(`inputs`, a handful of scalar settings) without literally being one.

`dc_bringup.launch.py`'s `build_bridge_and_mcap_actions()` reads that block at launch
time (not compiled in — editing the params file and relaunching is how you change it)
and, when `enabled: true`:

1. Renders a Vector `socket` sink (`mode = "tcp"`, `encoding.codec = "json"`,
   `framing.method = "newline_delimited"`) from `inputs`, converting each ROS topic to
   its public `dc.<tag>` route the same way the Bridge itself does, to
   `~/.dc/generated_mcap_sink.toml` — regenerated every launch, not meant to be
   hand-edited — and merges that path into whatever `dc_bridge.custom_config_files`
   the params file already lists (a second `parameters=[...]` entry for a list-valued
   ROS parameter *replaces* the file's value rather than appending to it, so this merge
   has to happen in the launch file's own Python, not by relying on `launch_ros`).
2. Starts `dc_mcap_writer` as a plain `ExecuteProcess` — deliberately not
   `ros2 run dc_mcap_writer dc_mcap_writer`: `ros2 run` spawns its target as a *child*
   of its own process and does not forward signals to it, so `launch`'s own
   respawn/shutdown handling would only ever reach the `ros2 run` wrapper, leaving the
   real `dc_mcap_writer` process running, orphaned, never getting the chance to finish
   its currently-open `.mcap` file. `dc_mcap_writer` has no `rclpy`/ROS-node dependency
   of its own, so nothing here actually needs `ros2 run`'s node-launching machinery —
   a sourced workspace already puts it on `PYTHONPATH`, so `python3 -m
   dc_mcap_writer.cli` runs it directly.

Like every passthrough sink, the generated Vector sink gets Vector's default in-memory
buffer, not the disk buffer blessed sinks get — see the "Passthrough:
`custom_config_files`" section of [Destinations](../destinations.md#passthrough-custom_config_files).

`dc_mcap_writer` itself routes each incoming Record to a Channel by its `tag` field
(falling back to `name`, then a fixed `dc.unknown` catch-all), and reads the Record's
normalized timestamp from its own `--time-key` (default `date`, matching the Bridge's
own `time_key` parameter — not currently exposed as its own `dc_mcap_writer:` field,
set it via `ros2 launch`'s underlying `ExecuteProcess` if you need a non-default value)
to set the MCAP message's `log_time`/`publish_time` — it understands all three
`time_format` values (`epoch_nanos`, `iso8601`, `double`), same as
[documented for Destinations](../destinations.md#time_format).

## Running `dc_mcap_writer` outside `dc_bringup`

The launch integration above covers the common case; `python3 -m dc_mcap_writer.cli`
(see `--help`) is still the standalone entrypoint underneath it, useful for converting
an existing NDJSON capture (`--stdin`) or running it against a hand-authored passthrough
snippet the way the [Elasticsearch tutorial](./elasticsearch.md) shows for other sinks.
