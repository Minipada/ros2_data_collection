# Vector is the blessed default shipper

Both upstream Fluent Bit and Vector can sit behind the Bridge's Forward-protocol boundary. We bless **Vector**: its postgres and aws_s3 (MinIO-compatible) sinks are native and maintained where Fluent Bit's pgsql output stayed experimental and MinIO required our own Go plugin; VRL replaces our four chained Lua/rewrite-tag filter hacks with one typed transform; it adds end-to-end acknowledgements on top of disk buffering; and it ships as a single static binary (x86_64/arm64). Docs, demos, and generated config target Vector only — Fluent Bit remains a drop-in option for memory-starved targets (~5MB vs ~100MB RSS) but is not documented as a first-class path.

## Consequences

- CI publishes a "vector-slim" prebuilt binary (feature-flagged build, ~30–40MB) for constrained deployments; users never compile Vector themselves.
- A `vector_vendor` ament package downloads the official static binary at build time, pinned to an exact version by checksum (arch-detected); a `vector_path` parameter allows using a system-installed Vector instead. Apt-repo install and Docker remain documented alternatives.

## Why the forward protocol is the Bridge→Shipper wire format

The Bridge must hand Records to the Shipper across a process boundary such that **the
sender knows what arrived** — durable ingest is impossible without receipt
acknowledgement. Requirements: a listener the Shipper already ships, sender-visible
acks, and a minimal Rust client. Vector's source menu, filtered:

- `socket`, `stdin` — one-way byte streams; no response direction exists, so the
  Shipper cannot confirm receipt. A Shipper restart silently loses in-flight Records.
- `file` — durable, but Vector neither deletes consumed files nor exposes its
  checkpoint, so the Bridge would rotate blind (delete too early = loss, too late =
  unbounded disk); fixing that means building an acknowledged queue anyway.
- websocket — Vector has a websocket sink, not a source.
- `http` — ack-capable (response deferred until the event is buffered) but costs an
  HTTP client plus batching logic; equal capability at a higher price.
- `vector` (native gRPC) — ack-capable; drags tonic/prost/protobuf codegen into the
  Bridge for no functional gain.
- `fluent` (Fluentd's open "Forward" spec) — ack-capable (`chunk`/`ack` options are in
  the spec), and the client side is ~170 lines of msgpack over TCP. **Chosen: the
  cheapest ack-capable listener.**

**The forward format is not Fluent Bit.** It is an open wire specification from the
CNCF Fluentd project; no Fluentd/Fluent Bit software runs, links, or vendors into
DC 2.0 (the embedded Fluent Bit is demolished separately, ADR-0001). DC docs call
this boundary the **shipper ingest protocol**; the name "fluent" appears only in the
generated Vector config (`type = "fluent"`) and interop documentation. A side effect,
not a goal: any forward-speaking receiver (including stock upstream Fluent Bit, ~5MB
RSS vs Vector's ~100MB) can be swapped in behind the boundary without Bridge changes.

## Amendment: confirmed delivery closes the acks half of the promise (#266)

The "sender-visible acks" requirement above was satisfied by *choosing* an ack-capable
protocol, but until #266 the Bridge's Forwarder never actually used the chunk/ack option
— every frame was sent bare. Spiked directly against the pinned Vector 0.57.0 binary
before writing any C++: a hand-rolled msgpack client sending `[tag, entries, {"chunk":
id}]` got back `{"ack": id}` even with the sink completely unreachable (connection
refused) and only a disk buffer engaged — confirming acks fire on durable-buffer-write,
not on final delivery to the Destination, which is the guarantee actually wanted here.
Also found: Vector 0.57 deprecates enabling `acknowledgements` on the source itself
in favor of the global `[acknowledgements] enabled = true` form (identical ack
behavior, no deprecation warning) — the renderer uses the global form.

A second on-disk queue for this window (mirroring the Uploader's intent queue, #265)
was considered and rejected in review: Vector's own disk buffer
(`shipper.buffer_max_bytes`) already covers sink/Destination outages, and the
`bridge_ready_gate` (ADR-0006) already covers "Vector isn't listening yet" at startup.
What was missing was purely the **in-flight** window — a Record already handed to a
`send()` call whose ack never arrives (Vector respawning, a TCP hiccup) — which an
in-memory (not disk-backed) bounded window closes cheaply. The double failure of "Bridge
crashes while this in-memory window is non-empty" is accepted as out of scope, same as
this ADR's original ready-gate/disk-buffer split assumed no double failures either.
