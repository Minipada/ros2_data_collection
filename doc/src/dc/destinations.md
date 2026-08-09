# Destinations

A **Destination** is an external system that receives Records or Files. In the DC 2.0
architecture (ADR-0003,
[`docs/adr/`](https://github.com/minipada/ros2_data_collection/tree/jazzy/docs/adr)), the
pluginlib destination-plugin layer is retired. The Bridge (`dc_bridge`) renders the
external Vector **Shipper's** configuration from plain ROS parameters for a **blessed
set** of Destination types — `postgres`, `s3`, `file`, `console` — and every other
destination in Vector's sink catalog is available through the **passthrough**: raw Vector
config snippets listed in the `custom_config_files` parameter, merged natively by Vector.

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
| `custom_config_files`       | Raw Vector config snippets (TOML) to merge — the passthrough                       | list\[str\] | `[]`                 |
| `vector_forward_host`       | Host the Shipper's ingest socket listens on                                        | str         | `"127.0.0.1"`        |
| `vector_forward_port`       | Port the Shipper's ingest socket listens on                                        | int         | `24224`              |
| `files.*`                   | Uploader settings; see [File uploads](#file-uploads-receives-files-the-uploader-adr-0005) | —     | —                    |

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

Type-specific parameters:

| Type       | Required                                    | Optional                                                                                                          |
| ---------- | ------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `postgres` | `user`, `password`, `database`, `table`     | `host` (default `127.0.0.1`), `port` (default `5432`)                                                             |
| `s3`       | `bucket`                                    | `region`, `endpoint`, `key_prefix`, `access_key_id` + `secret_access_key` (together), `force_path_style`, `batch_timeout_secs` |
| `file`     | `path` (Vector template syntax allowed)     |                                                                                                                    |
| `console`  |                                             |                                                                                                                    |

Invalid parameters (unknown `type`, missing required field, half a credential pair, an
out-of-range port…) are rejected with a clear error at Bridge startup — before Vector
is ever started.

## The `dc.<tag>` routing contract (public API)

The Bridge exposes one Shipper route per **Tag**. The Tag for a topic is its name with
the leading `/` dropped and the remaining `/` turned into `.`
(`/dc/measurement/uptime` → `dc.measurement.uptime`), and its route is named
**`dc.<tag>`** — the leading `dc.` names the Bridge's route transform, the rest is the
Tag verbatim (so `/dc/measurement/uptime`'s Records are consumable as
`dc.dc.measurement.uptime`). These names are **stable public API**: blessed sinks are
wired to them internally, and custom snippets consume them the same way. A route
exists for every topic listed in any configured Destination's `inputs`.

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
also checked with `vector validate` before the Shipper is started.

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
`deleted`, `content_type`, `size`, `duration`, `updated_at` — but the log is
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
