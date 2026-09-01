# External shipper process replaces embedded Fluent Bit

The Humble-era design embedded a forked Fluent Bit 2.1.3 as an in-process library, which forced us to maintain `fluent_bit_vendor` (source build of a patched fork), a custom C input plugin (`in_ros2`), and cgo-built Go output plugins — the dominant cause of install pain. For the Jazzy rewrite, the shipper runs as an **external process**, fed by a thin Bridge over the shipper ingest protocol. Install becomes "drop one prebuilt binary"; the fork, the vendor package, and the Go toolchain are deleted.

## Considered Options

- Keep embedding (rejected: the fork/build burden is inherent to embedding, not fixable)
- Native C++/Rust destination implementations with our own buffering (rejected: re-implements buffering/backpressure/retry that shippers already do well, and caps destination breadth)

## Consequences

- One extra process at runtime alongside the Bridge; negligible hop for JSON-sized Records.
- The Forward-protocol boundary makes the shipper swappable (Vector and Fluent Bit both consume it).

## Amendment (#440/#444): supervision is no longer always "localhost, by DC's bringup"

This ADR originally read "the shipper runs as an external process on localhost, supervised
by DC's bringup" without qualification. That is still the default (**managed mode**), but
it is no longer the only supported shape: in **unmanaged mode** (#444) the Bridge renders
the Shipper's config and connects over the shipper ingest protocol exactly as before, but
does not locate a binary, spawn it, or supervise it — an orchestrator does, and the Shipper
may run in its own container, possibly on a different host from the Bridge. See
[ADR-0015](./0015-split-deployment-topology.md) for why the decomposition stops at the
Shipper (and the Uploader) rather than extending further, and
[ADR-0014](./0014-uploader-runs-as-its-own-process.md) for the Uploader's own extraction
out of the Bridge process. Nothing about the shipper ingest protocol itself changes between
the two modes; only who starts and watches the Shipper does.
