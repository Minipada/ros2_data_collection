# External shipper process replaces embedded Fluent Bit

The Humble-era design embedded a forked Fluent Bit 2.1.3 as an in-process library, which forced us to maintain `fluent_bit_vendor` (source build of a patched fork), a custom C input plugin (`in_ros2`), and cgo-built Go output plugins — the dominant cause of install pain. For the Jazzy rewrite, the shipper runs as an **external process on localhost**, supervised by DC's bringup, fed by a thin Bridge over the Fluent Forward protocol. Install becomes "drop one prebuilt binary"; the fork, the vendor package, and the Go toolchain are deleted.

## Considered Options

- Keep embedding (rejected: the fork/build burden is inherent to embedding, not fixable)
- Native C++/Rust destination implementations with our own buffering (rejected: re-implements buffering/backpressure/retry that shippers already do well, and caps destination breadth)

## Consequences

- One extra supervised process at runtime; negligible localhost hop for JSON-sized Records.
- The Forward-protocol boundary makes the shipper swappable (Vector and Fluent Bit both consume it).
