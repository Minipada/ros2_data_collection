# New Measurements and blessed Destinations default to core, not Pro

A survey of tooling adjacent to DC (DDS/zenoh statistics, CAN bus health, Modbus
reachability, TimescaleDB/InfluxDB as time-series Destinations) turned up several
plausible additions to `dc_measurements` and `dc_bridge`. With `asaph_pro` now existing
as a separate closed-source repository of `asaph_*` Pro Packages (paid tier, discovered
by core at runtime via pluginlib — see `CONTEXT.md`'s **Pro Package** entry), every one
of these candidates raises the same question before any of them are worth scoping:
does a new Measurement or blessed Destination ship in `dc_*` (free, this repo) or
`asaph_*` (paid, `asaph_pro`)? Left unanswered, that question would otherwise be
re-litigated feature by feature.

**Decision: absent a stated reason to do otherwise, a new Measurement or blessed
Destination ships in core.** Pro is for capability that targets a specific paying
segment (e.g. a platform-specific integration such as MAVLink/PX4 drone telemetry),
not for general protocol- or system-health diagnostics any DC deployment could use —
`zenoh_stats`, `can_health`, and `modbus_health` are core by this default, the same tier
as the existing `fastdds_stats`/`cpu`/`network`/`thermal` Measurements they sit beside.

## Consequences

- A contributor proposing a new Measurement or blessed Destination does not need to
  ask "core or Pro?" case by case — core is the default, and only a specific,
  articulated reason (a platform-specific integration, or a deliberate monetization
  call) moves something into `asaph_pro` instead.
- `asaph_pro` stays reserved for capability that is genuinely segment-specific or a
  deliberate paid feature, rather than becoming the default landing spot for anything
  broadly useful — keeping the open half of open-core actually open.
- Moving a capability from core to Pro later is the harder direction: it revokes
  something existing users already have. This default is chosen knowing that
  asymmetry runs one way.

## Considered Options

- Decide placement case by case, per feature (rejected: no standing rule, so every
  future Measurement/Destination proposal reopens the same question this ADR closes).
- Default new capability to Pro unless explicitly justified as core (rejected: inverts
  the open-core value proposition — `dc_measurements` would stop being a genuinely
  useful telemetry pipeline on its own, which is the whole premise of an open core).
