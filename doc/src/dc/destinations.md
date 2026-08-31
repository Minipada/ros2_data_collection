# Destinations

A **Destination** is an external system that receives Records or Files. In the DC 2.0
architecture (ADR-0003,
[`docs/adr/`](https://github.com/minipada/ros2_data_collection/tree/jazzy/docs/adr)), the
pluginlib destination-plugin layer is retired. The Bridge (`dc_bridge`) renders the
external Vector **Shipper's** configuration from plain ROS parameters for a **blessed
set** of Destination types — `postgres`, `s3`, `file`, `console`, `vector` — and every
other destination in Vector's sink catalog is available through the **passthrough**: raw
Vector config snippets listed in the `custom_config_files` parameter, merged natively by
Vector.

Coming from DC 1.x and its `flb_*` destination plugins? See the
[migration guide](./migration.md).

```admonish abstract title="What is public API here"
Three things on this page are a stable contract you can build against, not
implementation detail:

1. **The configuration shape** — `destinations: [...]` plus one block per Destination
   with `type`, `receives`, `inputs`, and that type's own fields.
2. **The `dc.<tag>` routes** — one Shipper route per Tag, named deterministically from
   the topic. Passthrough snippets consume these.
3. **The File status Record** — the fields the Uploader writes under the `dc.files` Tag.

Everything else the renderer emits (component ids other than the reserved ones, the
exact TOML layout, Vector's own defaults) may change.
```

## Bridge node parameters

| Parameter                   | Description                                                                        | Type        | Default              |
| --------------------------- | ---------------------------------------------------------------------------------- | ----------- | -------------------- |
| `destinations`              | Names of the Destinations to enable                                                | list\[str\] | N/A (mandatory)      |
| `shipper.data_dir`          | Directory for the Shipper's persistent disk buffer                                 | str         | `"$HOME/.dc/buffer"` |
| `shipper.buffer_max_bytes`  | Disk-buffer size; Vector rejects anything below ~256 MiB                           | int         | Vector's minimum     |
| `shipper.managed`           | `true`: the Bridge locates, spawns and supervises the Shipper. `false` (unmanaged, #444): the Bridge only renders the config and connects — an orchestrator owns the Shipper's lifecycle | bool | `true` |
| `shipper.config_path`       | Where the rendered Shipper config is written (atomically: write then rename); shared with the Shipper container/pod in unmanaged mode | str | a temp-file path |
| `custom_config_files`       | Raw Vector config snippets (TOML) to merge — the passthrough                       | list\[str\] | `[]`                 |
| `vector_forward_host`       | Host the Shipper's ingest socket listens on                                        | str         | `"127.0.0.1"`        |
| `vector_forward_port`       | Port the Shipper's ingest socket listens on                                        | int         | `24224`              |
| `files.*`                   | Uploader settings; see [File uploads](#file-uploads-receives-files-the-uploader-adr-0005) | —     | —                    |

## Deployment modes: `shipper.managed`

`shipper.managed` picks who owns the Shipper's process lifecycle. The Bridge renders the
same config and connects over the shipper ingest protocol the same way in both modes —
only whether it also locates, spawns and supervises the Vector binary changes.

| | `shipper.managed: true` (default) | `shipper.managed: false` |
| --- | --- | --- |
| Shipper supervised by | the Bridge (`fork`/`exec` + `PR_SET_PDEATHSIG`) | an external orchestrator (container runtime, Kubernetes, systemd, …) |
| Vector binary | the vendored one, located on `AMENT_PREFIX_PATH` | not located at all — the orchestrator runs its own (e.g. the upstream Vector image) |
| `vector validate` at startup | yes, against the merged config | no — nothing to validate against without a binary |
| `shipper.config_path` | usually left at its temp-file default | set to a path on a volume shared with the Shipper container/pod |
| `~/ready` semantics | ready once the supervised Shipper accepts connections | identical: a TCP probe against `vector_forward_host:vector_forward_port`, independent of who spawned it |

**Use `shipper.managed: true` (the default) for:**

- **Single-robot / simulation / demos** — one process tree on one machine, `apt install`
  or a workspace build, nothing else to run or orchestrate. This is what every demo under
  [Demos](./demos.md) and the `dc_simulation` warehouse simulation use.
- **Local development and testing** — fewer moving parts: no container runtime, no
  volumes to wire up, `ros2 launch dc_bringup dc_bringup.launch.py` is the whole story.

Nothing about this mode changes with `shipper.managed` added — it is, and remains, the
default, and an existing deployment that never sets the parameter is unaffected.

**Use `shipper.managed: false` for:**

- **Multi-container / orchestrator-managed deployments** — Vector runs as its own
  container (the upstream image, not the vendored binary) under Compose, Podman Quadlet
  or Kubernetes, alongside the ROS/Bridge container on the same host. Set
  `shipper.config_path` to a path on a volume both containers mount, so the Bridge writes
  the config where the Shipper container reads it; the atomic write (below) is what makes
  that handoff safe even while the Shipper is already watching the file.
- **Per-component operations** — separate logs (no ROS/Vector log interleaving),
  independent restarts, and orchestrator-native resource limits/credentials for the
  Shipper, without changing anything about how the Bridge renders or validates its
  config.

This parameter is the Bridge-side building block for the larger split-deployment and
fleet topologies tracked in #440 (a separate `dc-uploader` entrypoint, a blessed `vector`
Destination type for robot→edge forwarding, etc.); those remain future work, not shipped
by `shipper.managed` alone.

### Atomic config write

The rendered config is always written atomically — a full write to
`<shipper.config_path>.tmp`, then `rename()` over the real path — in both modes. A reader
polling the path (an unmanaged Shipper container watching a shared volume; `vector
validate` in managed mode) can therefore never observe a partial write, regardless of how
large the config is or how the two containers' write/read timing lines up.

## Configuration contract

Every Destination is declared in the `destinations` list of the `dc_bridge` node's
parameters, with a parameter block named after it (see
`dc_bringup/params/dc_params.yaml` for a complete commented example):

```yaml
dc_bridge:
  ros__parameters:
    shipper:
      data_dir: "$HOME/.dc/buffer"   # Vector's disk-buffer directory
      # buffer_max_bytes: 268435488  # optional; Vector's disk-buffer minimum
    destinations: ["pgsql", "rustfs"]
    pgsql:
      type: postgres
      receives: records
      inputs: ["/dc/measurement/uptime"]   # ROS topics feeding this Destination
      host: "127.0.0.1"
      port: 5432
      user: "dc"
      password: "$DC_PG_PASSWORD"          # $VAR / ${VAR} env references are expanded
      database: "dc"
      table: "dc"
    rustfs:
      type: s3
      receives: records
      inputs: ["/dc/measurement/uptime"]
      bucket: "dc-records"
      endpoint: "http://127.0.0.1:9000"    # omit for AWS S3
      region: "us-east-1"
      access_key_id: "rustfsadmin"         # omit both to use ambient AWS credentials
      secret_access_key: "$DC_S3_SECRET"
      force_path_style: true               # path-style addressing for self-hosted stores
      key_prefix: "robot1/"
      batch_timeout_secs: 60               # object write interval; Vector default 300
```

The `s3` type works with any S3-compatible store. For self-hosting,
[RustFS](https://rustfs.com/) (Apache 2.0, S3-compatible, a drop-in MinIO
replacement) is the recommended choice — MinIO's community edition was archived
upstream in 2026 and no longer receives maintenance — but existing MinIO or Ceph RGW
deployments work identically: set `endpoint`, explicit credentials, and (typically)
`force_path_style: true`.

The `vector` type forwards to another Shipper over Vector's own native inter-instance
protocol (`type = "vector"` sink → `type = "vector"` source) — the standard way to chain
a robot's local Shipper to an edge aggregator's Shipper in a fleet deployment. `host` and
`port` name the downstream Shipper; unlike `postgres`, there is no default for either,
since there's no sensible address to assume for another Shipper.

```yaml
    to_aggregator:
      type: vector
      receives: records
      inputs: ["/dc/measurement/uptime"]
      host: "edge-aggregator.local"
      port: 6000
```

Common parameters for every blessed type: `type`, `receives` (`records` | `files`, see
the File uploads section below), `inputs` (ROS topic names), `time_key` (default
`date`) and `time_format` (default `epoch_nanos`) controlling the normalized timestamp
field written into each Record before routing.

### `time_format`

The Bridge forwards each Record's timestamp at full nanosecond resolution, taken from the
ROS message header. `time_format` decides how that is written into the Record:

| Value         | Written as                                     | Resolution kept |
| ------------- | ---------------------------------------------- | --------------- |
| `epoch_nanos` | integer nanoseconds since the epoch (default)  | nanosecond — exact |
| `iso8601`     | `2026-08-09T11:36:28.123456789` string         | nanosecond |
| `double`      | fractional seconds as a float64                | ~microsecond, rounds |

`double` is lossy by construction, not by implementation: a float64 has ~15–16 significant
digits and current epoch seconds already consume 10 of them. It remains available for
consumers that want a fractional-seconds column, but it cannot represent the resolution
the pipeline delivers.

```admonish warning
The destination's own column type can truncate independently of `time_format`. A
`double precision` column rounds an `epoch_nanos` value back off; PostgreSQL's native
`timestamptz` is microseconds internally; Elasticsearch's `date` type is milliseconds
(use `date_nanos` for the full value). Choose the column to match — `bigint` for
`epoch_nanos` is exact, sortable and indexable.
```

### `incident_id`

A Measurement configured for [incident capture](./measurements.md) tags every Record it
releases with the `incident_id` of the `FlushEvent` that released it. That field is part of
the Record envelope, so a `postgres` Destination writes it to its own **`incident_id`
column** — "everything from this one event" is `WHERE incident_id = '…'`, not a substring
search through a JSON payload:

```sql
ALTER TABLE dc ADD COLUMN incident_id text;
```

The Bridge renders the mapping into the `dc_bridge_normalize` transform for every `postgres`
Destination. Vector's `postgres` sink has no column options of its own — a top-level event
key lands in the same-named column and everything else is dropped — so the column has to
exist in the table before the first incident, exactly like every other column
(`tools/e2e/sql/init.sql` and `tools/infrastructure/docker/config/postgresql/init.sql` both
carry it). Records collected outside an incident have no `incident_id` and leave the column
NULL. Other Destination types need nothing: `incident_id` is already a top-level key of the
JSON they receive.

Type-specific parameters:

| Type       | Required                                    | Optional                                                                                                          |
| ---------- | ------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `postgres` | `user`, `password`, `database`, `table`     | `host` (default `127.0.0.1`), `port` (default `5432`)                                                             |
| `s3`       | `bucket`                                    | `region`, `endpoint`, `key_prefix`, `access_key_id` + `secret_access_key` (together), `force_path_style`, `batch_timeout_secs` |
| `file`     | `path` (Vector template syntax allowed)     |                                                                                                                    |
| `console`  |                                             |                                                                                                                    |
| `vector`   | `host`, `port`                              |                                                                                                                    |

Invalid parameters (unknown `type`, missing required field, half a credential pair, an
out-of-range port…) are rejected with a clear error at Bridge startup — before Vector
is ever started.

`$VAR` environment references are expanded by the Bridge in `shipper.data_dir`,
`custom_config_files`, and the `password` / `secret_access_key` credentials — and
**nowhere else**. In particular `file`'s `path` is handed to Vector verbatim, and Vector
does not expand environment variables there: `path: "$HOME/records.ndjson"` silently
writes to a literal `$HOME` directory beside the Bridge's working directory. Use an
absolute path (Vector creates missing parent directories).

## The `dc.<tag>` routing contract (public API)

The Bridge exposes one Shipper route per **Tag**. The Tag for a topic is its name with
the leading `/` dropped and the remaining `/` turned into `.`
(`/dc/measurement/uptime` → `dc.measurement.uptime`), and its route is named
**`dc.<tag>`** — the leading `dc.` names the Bridge's route transform, the rest is the
Tag verbatim (so `/dc/measurement/uptime`'s Records are consumable as
`dc.dc.measurement.uptime`). These names are **stable public API**: blessed sinks are
wired to them internally, and custom snippets consume them the same way. A route
exists for every topic listed in any configured Destination's `inputs`.

One route is not per-Tag: **`dc.dc.raw`** carries the whole `dc.raw.` Tag namespace when
[raw topic collection](./raw_topics.md) is enabled. Raw mode discovers topics — and so
mints Tags — while the Shipper is already running, which no fixed set of exact-Tag
branches can cover, so that branch matches on the Tag *prefix* instead. It is consumable
from a snippet exactly like the others; each event still carries its own `tag` field.

## Passthrough: `custom_config_files`

Any Vector sink type ships Records with zero DC code by consuming `dc.<tag>` routes
from a raw Vector config snippet (TOML):

```toml
# ~/.dc/custom_sinks.toml — an un-blessed sink type (http), pure Vector config
[sinks.my_http]
type = "http"
inputs = ["dc.dc.measurement.uptime"]   # the public dc.<tag> route
uri = "http://127.0.0.1:8080/ingest"
encoding.codec = "json"
```

```yaml
dc_bridge:
  ros__parameters:
    custom_config_files: ["$HOME/.dc/custom_sinks.toml"]
```

Snippets must not re-define component ids owned by the generated config (the
`dc_bridge_in` source, the `dc_bridge_normalize` and `dc` transforms, or any configured
Destination's name) or by another snippet. An invalid or colliding snippet is a loud
Bridge startup error naming the offending file; as a backstop, the merged config set is
also checked with `vector validate` before the Shipper is started. A snippet may define
transforms as well as sinks — only *defining* a reserved id is rejected, consuming one is
not.

A snippet cannot route a topic on its own. The Bridge derives both its ROS subscriptions
and its `dc.<tag>` route branches from `destinations`, and never reads a snippet's
`inputs` — so every topic a snippet consumes must also appear in some blessed
Destination's `inputs`, and `destinations` must name at least one (a `console` or `file`
Destination is the cheapest way to satisfy that). Two consequences follow from the
passthrough being outside the rendered config: the snippet's sink gets Vector's **default
in-memory buffer**, not the disk buffer `dc_bridge` gives every blessed sink, and it is
the snippet's job to make re-delivery idempotent if the store cares — the Shipper is
at-least-once (ADR-0002) either way.

The [Elasticsearch tutorial](./demos/elasticsearch.md) is the worked example for all of
this, end to end; the [InfluxDB demo](./demos/tb3_aws_influxdb.md) is the same mechanism
against a simulated robot. [MCAP recording](./demos/mcap_recording.md) (ADR-0009,
issue #210) is the same passthrough consumed by a standalone process instead of a
Vector-native sink — the shape to follow for any store Vector has no sink for at all.

## File uploads: `receives: files` (the Uploader, ADR-0005)

A Destination with `receives: files` (only `type: s3` qualifies) is served by the
Bridge's **Uploader**, not by a Vector sink: Records arriving on its `inputs` topics
are scanned for the `local_paths`/`remote_paths` File references Measurements embed
(camera, map, …), and each referenced File is uploaded (multipart + resumable for large
Files) to every `receives: files` Destination whose **name appears as a key in
the Record's `remote_paths`** — so the Destination name in the params file must match
the remote key the Measurement is configured with:

```json
{
  "name": "map",
  "local_paths":  { "yaml": "/tmp/map.yaml", "pgm": "/tmp/map.pgm" },
  "remote_paths": { "rustfs": { "yaml": "robot/map.yaml", "pgm": "robot/map.pgm" } }
}
```

Per File the Uploader: uploads (**multipart and resumable** for large Files — an
interrupted transfer picks up from the last completed part after a reconnect or Bridge
restart, using progress checkpointed under `<shipper.data_dir>/uploader/`), **verifies**
the object landed (`head` + size comparison), extracts metadata (content type, size,
and duration via ffprobe for `video/*` Files), and emits a **status Record** under the
`dc.files` Tag, routed like any Record to the Destination named by
`files.metadata_destination`. Status rows preserve the Humble `files_metrics` shape —
`group_name`, `robot_name`/`robot_id` (when present in the Record), `local_path`,
`remote_path` (`s3://bucket/key`), `storage_type`, `uploaded`, `on_filesystem`,
`deleted`, `content_type`, `size`, `duration`, `updated_at`, plus `thumbnail_path` when
[thumbnails](#thumbnails-optional-previews-for-image-and-video-files) are enabled — but
the log is
**append-only**: deletion appends a `deleted: true` row instead of updating one, and
PostgreSQL is only ever written through the Shipper's parameterized sink (no SQL
strings anywhere in DC).

Two guarantees consumers can rely on:

- **Group completion markers**: when one Record references several Files (map =
  pgm+yaml, camera batches), a `kind: group_complete` Record listing every File is
  emitted only after **all** of them are verified on **all** their Destinations — a
  consumer querying mid-upload sees per-File rows but no marker, and never has to guess
  whether a group fully arrived.
- **Deletion only after verified upload**: with `files.delete_when_sent: true`, a local
  File is removed only once it is verified on every Destination that receives it.
  Retries are idempotent — already-verified objects are not re-uploaded and status rows
  are never duplicated.

### Durability: the disk-backed intent queue

A Bridge restart never forgets a pending upload. Before a files-Destination's Record is
ever handed to the Uploader, it is written to disk as one **intent** — a JSON file
(`{version: 1, tag, timestamp, payload}`) at
`<shipper.data_dir>/queue/upload/<monotonic_ts>-<seq>.json`, via tmp-write-then-rename
(crash-atomic; no `fsync`, matching Humble Fluent Bit's own `storage.sync normal`). The
intent leaves the queue **only** when its Record has been fully processed — every File
verified everywhere, or reported missing — never on a timer, a cap, or a drop-oldest
policy: an upload intent and its Files live and die together. On startup every intent
left over from a previous run is replayed, oldest-first, alongside live traffic; because
processing is idempotent (an already-verified object short-circuits, multipart resumes
from its checkpointed sidecar), a replayed intent can never re-upload or duplicate a
status row.

A permanently-failing intent cannot starve the rest of the backlog: each intent gets its
own exponential backoff (5 s, doubling, capped at 2000 s — Humble Fluent Bit's own
scheduler defaults) and the worker sweeps oldest-first, skipping whatever is still
backing off rather than retrying the same head-of-line entry forever. The queue's depth
is surfaced as a cheap observability hook in the `~/ready` service's message text.

This is **at-least-once**, not exactly-once: a crash between a status Record being
forwarded and its intent being acked can replay that status Record after restart (the
Uploader's own dedup set is in-memory, so a fresh process may re-emit one it already
sent). Consumers should key on the latest row per `(local_path, storage_type)`, same as
the Records path.

A Record referencing no Files is enqueued and acked in the same pass with no retries, so
it never lingers in the queue's steady state. Note that `base64`-heavy Records (e.g. a
map's `save_base64: on`) inline file content directly into the payload and so consume
queue-directory bytes proportionally faster than a plain File reference.

### Thumbnails: optional previews for image and video Files

A dashboard drawing a grid of inspection photos otherwise pulls every full-size camera
JPEG — and every video file — just to render tiles. With `files.thumbnails` enabled, the
Uploader derives a small JPEG preview from each image or video File and uploads it next
to the original as `<remote_path>.thumb.jpg`, recording it in that File's status Record
as `thumbnail_path` (same `s3://bucket/key` shape as `remote_path`; the field is simply
absent when there is no preview).

**Default is off**, and enabling it cannot change what happens to your data:

```yaml
dc_bridge:
  ros__parameters:
    files:
      thumbnails:
        enabled: true
        max_dimension: 320       # bound on the preview's longest side, in pixels
        ffmpeg_binary: "ffmpeg"  # override if it isn't on PATH
```

A preview is a **derived** artefact and is treated as strictly secondary to the File it
comes from:

- It is generated only **after** the original is uploaded and verified on that
  Destination, never before or in parallel.
- Every failure — no `ffmpeg` on `PATH`, an undecodable File, a store that rejects the
  preview — is swallowed. It never fails, delays, retries, or blocks the primary upload;
  the File uploads exactly as it would with the feature off, and the only visible
  difference is a missing `thumbnail_path` plus a Bridge warning counting how many
  previews couldn't be generated.
- Local previews are scratch files under `<shipper.data_dir>/uploader/thumbs/`, removed
  before the Record finishes processing. So a preview never joins the un-uploaded pool
  [retention](#retention-bounded-local-storage-for-un-uploaded-files) measures and can
  never outlive its original: an intent retention sheds never uploaded, so no preview
  was ever derived from it.
- Remote previews are idempotent by key. A Bridge restart mid-upload replays the intent,
  finds the preview already on the store, and neither re-decodes nor re-uploads it.

`max_dimension` bounds the **longest side**; aspect ratio is preserved and a File already
smaller than the bound is never upscaled. Images become a scaled still; videos become
their first frame. Files that are neither (map YAML, logs) are skipped entirely rather
than handed to a decoder.

**The decode dependency**: previews are generated by shelling out to the **ffmpeg CLI**,
the same toolchain `files.ffprobe_binary` already uses for the `duration` column — one
tool covering both stills and video first-frames, kept out of the Bridge's link line, and
crash-isolated in a child process. `dc_bridge/package.xml` declares `ffmpeg` as an
`exec_depend`, so `rosdep install` provides it; if it is missing at runtime the Bridge
still uploads normally and simply produces no previews.

### Retention: bounded local storage for un-uploaded Files

The intent queue above never abandons an intent by design — an intent and its Files
live and die together, and deleting the only copy of data as a side effect of queue
management is forbidden. That means a robot offline for weeks, or pointed at a store
that's unreachable for weeks, accumulates un-uploaded Files on disk **without bound**,
same as Humble did. A full disk takes down more than DC, so `files.retention` is the
**one, explicit, opt-in, audited** mechanism allowed to abandon data under disk
pressure — nothing else in the system deletes a File it hasn't verified was uploaded.

**Default is off**: with `files.retention` absent (or both limits at their defaults),
behavior is unchanged — accumulation, matching Humble. Configuring it is a deliberate
choice to shed data rather than run out of disk:

```yaml
dc_bridge:
  ros__parameters:
    files:
      retention:
        max_bytes: 10737418240   # cap on the un-uploaded Files pool; 0/absent = unlimited (default)
        max_age_days: 30         # optional; whichever limit is hit first
```

Scope is exactly the pool of Files referenced by intents still pending in
`<shipper.data_dir>/queue/upload/` — the same disk-backed queue the durability section
above describes. A File the Uploader has already verified everywhere but is still
waiting to physically delete (`files.delete_when_sent`, e.g. after a filesystem error)
is **never** retention's victim; that's `delete_when_sent`'s own retry loop to resolve,
not retention's.

When either limit is exceeded, the Bridge sheds the **oldest** eligible intent — its
File(s) and its queue entry, always together, never one without the other — and repeats
until back under both limits. Each shed File gets its own audit row through the normal
`dc.files` metadata path: `deleted: true, uploaded: false` is the queryable "shed
without upload" signature, distinct from a normal `delete_when_sent` deletion (which is
always `uploaded: true`, since that path only ever deletes a File already confirmed on
every Destination). Emitting the audit row is best-effort — a Shipper outage doesn't
block the shed itself — but every shed always logs a rate-limited warning, since data
was just abandoned without ever leaving the robot.

A Destination named by `metadata_destination` may declare **no `inputs` at all** — being
named there is itself what routes the `dc.files` Tag to it. That is the usual shape when
the status log lives in its own table: one `postgres` Destination with `inputs` for the
Records, and a second one with no `inputs` for the File status log.

`inputs` is otherwise mandatory: a Destination that neither lists a topic nor receives
`dc.files` would have nothing to deliver, and is rejected at Bridge startup.
