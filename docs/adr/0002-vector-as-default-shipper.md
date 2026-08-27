# Vector is the blessed default shipper

Both upstream Fluent Bit and Vector can sit behind the Bridge's Forward-protocol boundary. We bless **Vector**: its postgres and aws_s3 (MinIO-compatible) sinks are native and maintained where Fluent Bit's pgsql output stayed experimental and MinIO required our own Go plugin; VRL replaces our four chained Lua/rewrite-tag filter hacks with one typed transform; it adds end-to-end acknowledgements on top of disk buffering; and it ships as a single static binary (x86_64/arm64). Docs, demos, and generated config target Vector only — Fluent Bit remains a drop-in option for memory-starved targets (~5MB vs ~100MB RSS) but is not documented as a first-class path.

## Consequences

- A `vector_vendor` ament package vendors the official static binary, pinned to an exact version by checksum (arch-detected); a `vector_path` parameter allows using a system-installed Vector instead. Apt-repo install and Docker remain documented alternatives.

## Amendment: checked-in binary instead of a build-time download (#424)

`vector_vendor` originally fetched its pinned binary over the network at build time via
`file(DOWNLOAD ...)`, which violates the ROS buildfarm's no-network-access policy for
binarydeb jobs and defeats #423's network-isolated build check. The "vector-slim"
feature-flagged build this ADR originally proposed CI would publish was never actually
built — no such CI job exists in this repo — so it is not available to vendor either.

Fixed by checking the official `vectordotdev/vector` release tarballs (x86_64 and
aarch64, same bytes `file(DOWNLOAD ...)` used to fetch, same pinned SHA256 checksums)
directly into `vector_vendor/prebuilt/`; `CMakeLists.txt` now extracts and installs from
the local file, performing no network I/O. The *tarball*, not the extracted binary, is
what's committed: the extracted `vector` binary is 120–142MB (stripped/unstripped)
depending on target, over GitHub's 100MB hard per-file limit, while the release
tarball's upstream gzip compression brings each architecture's file to ~50-54MB — under
the limit.

**Not tracked via Git LFS**, per #421's original implementation decision ("no
release-time injection step, no `git-lfs`: committed like any other tracked file").
LFS was evaluated and briefly adopted mid-review: the specific concern that motivated
#421's original call — that LFS content wouldn't survive `bloom`'s release-tarball
export step — turned out to be unfounded. `bloom`'s `export_upstream` delegates to
`vcstools.GitClient.checkout()` (a plain `git clone` + `git checkout <tag>`, which does
run LFS's smudge filter), and the export machinery itself
(`vcstools.git_archive_all.GitArchiver`) reads files off the working-tree filesystem
rather than through `git archive`'s blob-store plumbing (the actual, narrower reason
plain `git archive` breaks LFS) — confirmed empirically against a throwaway LFS repo and
the real `vcstools`/`bloom` code, not just read.

LFS was reverted anyway for a more basic reason: GitHub's free LFS tier is 1GB storage
and 1GB bandwidth per month, and every future Vector version bump adds both tarballs
(~106MB) as new, non-deduplicated LFS objects — roughly 9 version bumps before storage
alone exhausts the free tier, independent of and sooner than the plain-blob approach's
own cost (git history growing by the same ~106MB per bump, but against no comparable
quota). A plain committed blob has no such ceiling; the tradeoff is that every future
Vector bump grows this repo's ordinary git history by ~106MB, permanently, since git
does not deduplicate binary blobs across versions. Revisit if/when that accumulation
becomes the more pressing cost — the LFS path is proven to work, should it be needed.

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
