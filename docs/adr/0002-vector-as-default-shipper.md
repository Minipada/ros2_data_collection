# Vector is the blessed default shipper

Both upstream Fluent Bit and Vector can sit behind the Bridge's Forward-protocol boundary. We bless **Vector**: its postgres and aws_s3 (MinIO-compatible) sinks are native and maintained where Fluent Bit's pgsql output stayed experimental and MinIO required our own Go plugin; VRL replaces our four chained Lua/rewrite-tag filter hacks with one typed transform; it adds end-to-end acknowledgements on top of disk buffering; and it ships as a single static binary (x86_64/arm64). Docs, demos, and generated config target Vector only — Fluent Bit remains a drop-in option for memory-starved targets (~5MB vs ~100MB RSS) but is not documented as a first-class path.

## Consequences

- CI publishes a "vector-slim" prebuilt binary (feature-flagged build, ~30–40MB) for constrained deployments; users never compile Vector themselves.
- A `vector_vendor` ament package downloads the official static binary at build time, pinned to an exact version by checksum (arch-detected); a `vector_path` parameter allows using a system-installed Vector instead. Apt-repo install and Docker remain documented alternatives.
