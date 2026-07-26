# Overview

## DC 2.0 (Jazzy): blessed Destinations + passthrough

In the DC 2.0 architecture (ADR-0003, `docs/adr/`), the pluginlib destination-plugin
layer is retired. The Bridge (`dc_bridge`) renders the external Vector Shipper's
configuration from plain ROS parameters for a **blessed set** of Destination types —
`postgres`, `s3`, `file`, `console` — and every other destination in Vector's sink
catalog is available through the **passthrough**: raw Vector config snippets listed in
the `custom_config_files` parameter, merged natively by Vector.

### Configuration contract

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
`date`) and `time_format` (`double` | `iso8601`, default `double`) controlling the
normalized timestamp field written into each Record before routing.

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

### The `dc.<tag>` routing contract (public API)

The Bridge exposes one Shipper route per **Tag**. The Tag for a topic is its name with
the leading `/` dropped and the remaining `/` turned into `.`
(`/dc/measurement/uptime` → `dc.measurement.uptime`), and its route is named
**`dc.<tag>`** — the leading `dc.` names the Bridge's route transform, the rest is the
Tag verbatim (so `/dc/measurement/uptime`'s Records are consumable as
`dc.dc.measurement.uptime`). These names are **stable public API**: blessed sinks are
wired to them internally, and custom snippets consume them the same way. A route
exists for every topic listed in any configured Destination's `inputs`.

### Passthrough: `custom_config_files`

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

### File uploads: `receives: files` (the Uploader, ADR-0005)

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

The `files` block:

```yaml
dc_bridge:
  ros__parameters:
    files:
      delete_when_sent: true         # default false
      metadata_destination: "pgsql"  # required with any receives: files destination;
                                     # must name a receives: records destination
      ffprobe_binary: "ffprobe"      # optional; video duration probe
      # multipart_threshold_bytes: 16777216  # optional; multipart above this size
      # multipart_part_size_bytes: 8388608   # optional; >= 5 MiB for real S3 stores
```

