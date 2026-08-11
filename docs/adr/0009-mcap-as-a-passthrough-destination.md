# MCAP recording is a passthrough consumer, not a blessed Destination

Issue #210 asks for Records to be recordable as `.mcap` files so they can be replayed
with ROS 2 bag tooling (`ros2 bag info`, Foxglove). Vector — the DC 2.0 Shipper (ADR-0002)
— has no MCAP sink, so this cannot be a config-only addition the way `postgres`/`s3`/
`file`/`console` were; ADR-0003 gives two shapes and asks that the choice be recorded
before implementing.

**Decision: passthrough (`dc_mcap_writer`), not a Bridge-side blessed Destination.**

`dc_mcap_writer` is a small standalone process (ament_python package, no ROS
dependencies of its own) that consumes Records the same way every other non-blessed
sink does — over the public `dc.<tag>` routes (ADR-0003), via a `custom_config_files`
passthrough snippet using Vector's `socket` sink (TCP, newline-delimited JSON) pointed
at it. It registers one JSON-schema Channel per Tag (the pattern in the linked
`foxglove/mcap` `jsonschema/writer.cpp` example) and rotates to a new `.mcap` file by
size or elapsed time, whichever comes first.

## Why not the Bridge-side option

Local-first recording that survives a Shipper outage — the one stated justification in
#210 for putting MCAP writing inside the Bridge — is not a requirement here: DC's
Shipper is already at-least-once and disk-buffered per blessed sink (ADR-0002), so a
robot losing connectivity does not lose Records either way, it just delays them. Adding
MCAP writing to the Bridge would put a second delivery path (own file handles, own
rotation policy, own failure modes) next to the Vector handoff, permanently, for a
capability every other non-blessed destination gets by being outside the Bridge
entirely (#246). That asymmetry is exactly what ADR-0003 exists to avoid.

## Consequences

- No `dc_bridge` code changes: `destinations` still only names blessed sinks
  (`console`/`file`/… ) to route the topics MCAP recording needs onto their `dc.<tag>`
  routes, same as the Elasticsearch and InfluxDB passthrough demos.
- `dc_mcap_writer` gets Vector's default in-memory sink buffer, not the Bridge's disk
  buffer — consistent with every other passthrough sink (see the "Passthrough:
  `custom_config_files`" section of `doc/src/dc/destinations.md`), not a special case
  for MCAP.
- Rotation and retention are `dc_mcap_writer`'s own concern, not `dc_bridge`'s — there is
  no dependency on the Files retention policy (#267), which governs the Bridge's
  Uploader intent queue, an unrelated code path.
- If local-first recording that survives a Shipper outage later becomes a real
  requirement, that is grounds to revisit this decision, not to extend
  `dc_mcap_writer` to fake it (e.g. its own persistent queue) — the honest answer at
  that point is a Bridge-side writer per the rejected option below.

## Considered Options

- Bridge-side blessed Destination (`dc_bridge` writes MCAP itself, bypassing Vector) —
  rejected: no stated requirement for local-first durability past what the Shipper
  already gives every blessed sink, and it would duplicate a delivery path inside the
  Bridge indefinitely for one sink type.
