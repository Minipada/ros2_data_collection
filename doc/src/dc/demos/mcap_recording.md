# MCAP recording (passthrough)

`dc_bridge` blesses exactly four Destination types — `postgres`, `s3`, `file`, `console`
(see [Destinations](../destinations.md)) — and MCAP is not one of them; Vector, the
Shipper, has no MCAP sink at all. This tutorial is the worked example for #210: the
ADR-0003 **passthrough** plus a small standalone process, `dc_mcap_writer`
(ADR-0009, [`docs/adr/`](https://github.com/minipada/ros2_data_collection/tree/jazzy/docs/adr)),
that consumes the same public `dc.<tag>` routes a blessed Destination consumes and
writes them as rotated `.mcap` files, ready to open with `ros2 bag info` or
[Foxglove](https://foxglove.dev/).

Hardware-free: four system Measurements, one Vector passthrough snippet, one extra
terminal for `dc_mcap_writer`.

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

## One-time: build `dc_mcap_writer`

`dc_mcap_writer` is a plain `ament_python` package with one Python dependency
(`mcap`), already listed in the workspace's `requirements.txt`/`pyproject.toml`:

```bash
colcon build --packages-select dc_mcap_writer
```

## One-time: install the passthrough sink config

Because there is no MCAP Destination to configure through ROS parameters, the Vector
sink itself ships as a plain file in this package, `dc_demos/config/mcap_sink.toml`,
installed to the package's share directory. Copy it into place once before the first
launch:

```bash
mkdir -p ~/.dc
cp "$(ros2 pkg prefix dc_demos)/share/dc_demos/config/mcap_sink.toml" ~/.dc/
```

The params file's `custom_config_files` points at that path. Editing the copy in
`~/.dc/` is how you change the sink — it is read at Bridge startup, not compiled in.

## Run it

Start `dc_mcap_writer` first — it listens on a TCP socket, and Vector's `socket` sink
(re)connects to it, so having a listener up before the Bridge starts means the very
first Record lands instead of being retried:

```bash
mkdir -p ~/dc_mcap_out
python3 -m dc_mcap_writer.cli --output-dir ~/dc_mcap_out --max-duration-secs 300
```

```admonish warning
Use `python3 -m dc_mcap_writer.cli`, not `ros2 run dc_mcap_writer dc_mcap_writer`, in
any script or supervisor that stops it with a targeted signal rather than an
interactive Ctrl-C. `ros2 run` starts the target as a *child* of its own process and
does not forward signals to it — a `kill` on `ros2 run`'s own PID leaves the real
`dc_mcap_writer` process running, orphaned, and it never gets the chance to finish
its currently-open `.mcap` file (see the Rotation section below). An interactive
Ctrl-C is unaffected — the terminal sends SIGINT to the whole foreground process
group, which reaches both — but anything backgrounding it via `&` and killing it by
PID, a systemd unit, or a container entrypoint, hits this. `dc_mcap_writer` has no
rclpy/ROS-node dependency of its own, so nothing here actually needs `ros2 run`'s
node-launching machinery — a sourced workspace already puts it on `PYTHONPATH`.
```

Then, in a second terminal:

```bash
colcon build
ros2 launch dc_demos mcap_recording.launch.py
```

The `console` Destination prints every Record as it is shipped, so the terminal doubles
as a local view of what `dc_mcap_writer` is receiving; the writer's own terminal logs
each file it opens and closes.

## Verify the recording

Stop both processes (`Ctrl-C`) once a few Records have been collected — `dc_mcap_writer`
finishes the file it has open on shutdown, so every `.mcap` under `~/dc_mcap_out` is
independently valid, not just the most recent rotation:

```bash
ros2 bag info ~/dc_mcap_out/records_<timestamp>_0001.mcap
```

```
Files:             records_20260811T090626Z_0001.mcap
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

`dc_mcap_writer` rotates to a new `.mcap` file once the current one hits `--max-bytes`
(default 128 MiB) or has been open `--max-duration-secs` (default 300s), whichever
comes first — the same "whichever limit first" shape used for `files.retention`
(ADR-0005), applied here to `dc_mcap_writer`'s own output rather than the Bridge's
Uploader queue (ADR-0009 explains why this stays outside `dc_bridge`). Filenames are
`<prefix>_<UTC timestamp>_<pid>_<rotation index>.mcap`; the PID and counter together
guarantee a unique name even across a process restart landing in the same wall-clock
second as the previous process's last rotation — without both, the new process could
compute the identical name and its `open(..., "wb")` would silently truncate the file
the previous process had already finished.

```admonish warning
A `.mcap` file only becomes readable once its rotation finishes — writing the file's
closing footer is what `--max-bytes`/`--max-duration-secs` triggers, not something
every individual Record write does. A process that never gets to shut down gracefully
(`SIGKILL`, or a container/orchestrator grace period too short for the clean-shutdown
path to complete) loses whatever is in the file still open at that moment; every
already-finished rotation stays valid and readable regardless. `--max-duration-secs`'s
default balances this against not producing too many small files — lower it for
tighter durability, raise it for fewer files, per your own tolerance for that
loss window.
```

## Understanding the configuration

```admonish info
See the [Elasticsearch tutorial](./elasticsearch.md) for the full passthrough
mechanics — this section only covers what is specific to MCAP.
```

`dc_demos/config/mcap_sink.toml` declares one Vector `socket` sink, `mode = "tcp"`,
`encoding.codec = "json"`, `framing.method = "newline_delimited"`, consuming the four
`dc.dc.measurement.*` routes and streaming them as newline-delimited JSON to
`dc_mcap_writer`'s listener (`--host`/`--port`, default `127.0.0.1:9191`, matching the
snippet's `address`). Like every passthrough sink, it gets Vector's default in-memory
buffer, not the disk buffer blessed sinks get — see the "Passthrough:
`custom_config_files`" section of [Destinations](../destinations.md#passthrough-custom_config_files).

`dc_mcap_writer` routes each incoming Record to a Channel by its `tag` field (falling
back to `name`, then a fixed `dc.unknown` catch-all), and reads the Record's normalized
timestamp from `--time-key` (default `date`, matching the Bridge's own `time_key`
parameter) to set the MCAP message's `log_time`/`publish_time` — it understands all
three `time_format` values (`epoch_nanos`, `iso8601`, `double`), same as
[documented for Destinations](../destinations.md#time_format).
