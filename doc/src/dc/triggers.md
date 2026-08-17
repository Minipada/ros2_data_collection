# Triggers

## Description

A **Trigger** fires a one-shot signal — a `dc_interfaces/msg/FlushEvent` — when a composition
of [Conditions](./conditions.md) goes from false to true. Measurements listening for that
event release the window of recent data they have been holding back, so the seconds *leading
up to* an event are collected, not just the seconds after it.

The motivating case is incident review: when an autonomous robot emergency-brakes, what
matters is what happened before the brake. A Condition cannot express that — it gates
collection going forward, and by the time it turns true the run-up is gone. A Trigger can,
because the Measurement was already buffering.

## Trigger vs Condition

Both are described with the same `if_all`/`if_any`/`if_none` Condition lists, and a Trigger is
built out of Condition plugins — but they answer different questions.

|                         | Condition                                                     | Trigger                                                                    |
| ----------------------- | ------------------------------------------------------------- | -------------------------------------------------------------------------- |
| Question it answers     | "Should this Measurement be collecting *right now*?"          | "Did something just happen?"                                               |
| Shape of the signal     | A level: true for as long as the predicate holds              | An edge: fires once on the false→true transition                           |
| Effect on collection    | Gates live publishing while true                              | Releases a pre-collected buffer, once                                      |
| Data it can give you    | Everything *from* the moment it turned true                   | Everything from the `buffer_duration_sec` *before* the event, plus post-roll |
| Where it is configured  | Per Measurement (`if_all_conditions`, …)                      | On the `trigger_broadcast_node`, once for the whole robot                  |
| How Measurements see it | Directly, as their own gate                                   | Indirectly, as a `FlushEvent` on a topic several Measurements can share    |
| Scope                   | One Measurement                                               | Every Measurement subscribed to its `flush_topic`                          |

They compose: a Trigger is *made of* Conditions, and a buffering Measurement can still be
gated by its own Conditions. Nothing about a Trigger changes what a Condition means.

```admonish warning title="Trigger is a specific word here"
Before this feature existed, "trigger" was loose talk for "a Condition turning on", and the
[glossary](./concepts.md#glossary) told you to avoid the word. It now names a real, distinct
plugin type. Say **Condition** for a gate that stays true, and **Trigger** only for the
edge-firing plugin documented here.
```

## The flush broadcast

The `trigger_broadcast_node` loads the Condition plugins named in `condition_plugins` (the same
Condition plugins Measurements use) plus one Trigger plugin, polls the Trigger on a timer, and
publishes a `FlushEvent` on a configurable topic (`/dc/flush` by default) each time it fires:

```
                    ┌──────────────────────┐
   /odom, … ───────▶│  Condition plugins   │
                    └──────────┬───────────┘
                               │ if_all / if_any / if_none
                    ┌──────────▼───────────┐
                    │  Trigger (EdgeTrigger)│  false → true?
                    └──────────┬───────────┘
                               │ FlushEvent { incident_id, stamp }
                    ┌──────────▼───────────┐
                    │      /dc/flush       │
                    └───┬──────────────┬───┘
                        │              │
              ┌─────────▼────┐  ┌──────▼───────┐
              │ Measurement A│  │ Measurement B│   release their buffered windows,
              │  (buffering) │  │  (buffering) │   both tagged with the same incident_id
              └──────────────┘  └──────────────┘
```

The node mints a fresh `incident_id` (a UUID) for every firing; nothing else generates one, so
every subscriber of one event shares one ID. Several Measurements can point at the same
`flush_topic`, which is the whole reason the signal is broadcast on a topic rather than wired
per Measurement: adding a sensor stream to an existing incident-capture setup needs no change
to the Trigger.

```admonish note title="Only self-updating Conditions are meaningful here"
A Measurement passes its own freshly-collected Record to each Condition it consults, so
plugins like `BoolEqual` read a field out of that Record. A Trigger has no Record of its
own: it only makes sense to compose Conditions that maintain their own state from a
subscription, such as [Moving](./conditions/moving.md).
```

```admonish info title="Running the broadcast node"
`trigger_broadcast_node` is a lifecycle node, like the Measurement server, but it is not part
of `dc_bringup`'s launch file yet — run it alongside the rest of the stack and transition it
yourself (or add it to your own launch file and lifecycle manager `node_names`):

    ros2 run dc_triggers trigger_broadcast_node --ros-args --params-file <your_params.yaml>

A Measurement subscribes to a plain topic, so anything publishing a `FlushEvent` on
`flush_topic` releases the buffer — including `ros2 topic pub`, which is how the
`tools/e2e/scripts/run_incident.sh` scenario drives a flush.
```

## `FlushEvent`

`dc_interfaces/msg/FlushEvent` is the entire contract between the Trigger side and the
Measurement side:

| Field         | Type                     | Description                                                        |
| ------------- | ------------------------ | ------------------------------------------------------------------ |
| `incident_id` | string                   | UUID minted by the broadcast node, one per firing                  |
| `stamp`       | builtin_interfaces/Time  | When the Trigger fired                                             |

A Measurement adopts the `incident_id` it receives; it never generates its own. An event that
arrives while a Measurement is already flushing, in post-roll, or in cooldown is ignored.

## `incident_id`

`incident_id` is a top-level field of the Record envelope, beside `tags`, `run_id` and `name`
— not a key nested inside the measurement's own data. Every Record and File released by one
flush cycle carries the same value, so "everything from this one event" is a single query
rather than a timestamp range reconstructed by hand:

```sql
SELECT * FROM dc_records WHERE incident_id = '3f2b1c7e-…' ORDER BY date;
```

- A `postgres` Destination writes it to its own **`incident_id` column** — the column must
  exist in the table beforehand; see [Destinations](./destinations.md#incident_id).
- A Record collected outside an incident carries no `incident_id` at all, leaving the column
  NULL.
- A [Group](./groups.md) lifts a member's `incident_id` onto the merged Record the same way it
  does `tags`, so grouping does not bury it.
- Other Destination types need no configuration: `incident_id` is already a top-level key of
  the JSON they receive.

## Available plugins

| Name                              | Description                                                              |
| ---------------------------------- | -------------------------------------------------------------------------- |
| [Edge trigger](./triggers/edge_trigger.md) | Fires once on the false→true rising edge of an `if_all`/`if_any`/`if_none` Condition composition |

## Node parameters

| Parameter name     | Description                                | Type(s)     | Default          |
| ------------------- | --------------------------------------------- | ----------- | ---------------- |
| condition_plugins   | Name of the condition plugins to load       | list\[str\] | N/A (mandatory)  |

## Plugin parameters

Every Trigger plugin loaded by this node is namespaced under `trigger` and shares these
parameters:

| Parameter name              | Description                                                     | Type(s)     | Default        |
| ---------------------------- | -------------------------------------------------------------------- | ----------- | -------------- |
| trigger.plugin               | Name of the trigger plugin to load                                | str         | N/A (mandatory) |
| trigger.if_all_conditions    | Fire only once every named Condition is active                    | list\[str\] | []             |
| trigger.if_any_conditions    | Fire once any named Condition is active                           | list\[str\] | []             |
| trigger.if_none_conditions   | Fire only once no named Condition is active                       | list\[str\] | []             |
| trigger.topic                | Topic `FlushEvent` messages are published on                      | str         | "/dc/flush"    |
| trigger.polling_interval     | Interval in milliseconds at which the composed Condition state is checked | int | 100            |

## Measurement parameters

The other half of the feature lives on each Measurement: what it buffers, and what it does
once a `FlushEvent` releases it. All five are optional, and a Measurement that leaves
`buffer_duration_sec` at 0 behaves exactly as it always has — this feature is entirely
opt-in. They are documented in full, with the state machine and how Files are staged, in
[Measurements](./measurements.md#plugin-parameters).

| Parameter name             | Description                                                                                       | Type(s) | Default     |
| -------------------------- | ------------------------------------------------------------------------------------------------- | ------- | ----------- |
| buffer_duration_sec        | Seconds of history to hold instead of publishing live; 0 disables buffering entirely              | float   | 0           |
| post_roll_duration_sec     | Seconds to keep publishing live after the release finishes, still tagged with the same `incident_id`; 0 means pre-roll only | float | 0 |
| cooldown_sec               | Seconds to ignore further `FlushEvent`s once post-roll ends, before buffering re-arms itself; 0 re-arms immediately | float | 0 |
| max_flush_rate_hz          | Ceiling in Records per second on how fast the released window is emitted; 0 releases it in one burst | float | 0           |
| flush_topic                | Topic the `FlushEvent` that releases this Measurement is received on                              | str     | "/dc/flush" |

An armed Measurement moves through four states per incident — **Buffering** (accumulate,
publish nothing) → **Flushing** (release the window, oldest first, rate-limited) →
**PostRoll** (publish live, same `incident_id`) → **Cooldown** (ignore further events) → back
to Buffering, with no manual re-arm.

```admonish tip title="Sizing max_flush_rate_hz"
A Measurement's data publisher is `KeepLast(1)`. A whole window published back-to-back in one
callback is coalesced down to its last Record before any subscriber runs, so a burst release
(`max_flush_rate_hz: 0`) only delivers the full window to a subscriber that keeps up. Set a
rate comfortably above the collection rate being released — 20 Hz for a 2 Hz Measurement — and
the window is paced out intact, which is also what keeps a backlog dump from competing with
live collection right after an incident.
```

## Example

An emergency-brake capture: the robot buffers the last 10 seconds of uptime data, and the
moment it starts moving the buffer is released and collection continues live for 5 more
seconds. Swap the `moving` Condition for whatever expresses your event.

`trigger_broadcast_node`'s parameters:

```yaml
trigger_broadcast_node:
  ros__parameters:
    condition_plugins: ["moving"]

    moving:
      plugin: "dc_conditions/Moving"
      odom_topic: "/odom"

    trigger:
      plugin: "dc_triggers/EdgeTrigger"
      if_all_conditions: ["moving"]
      topic: "/dc/flush"
      polling_interval: 100
```

The Measurements listening for it:

```yaml
measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime", "memory"]

    # Armed: publishes nothing until a FlushEvent arrives on /dc/flush, then releases the
    # last 10 seconds of collection tagged with that event's incident_id.
    uptime:
      plugin: "dc_measurements/Uptime"
      polling_interval: 500
      group_key: "uptime"
      buffer_duration_sec: 10.0
      post_roll_duration_sec: 5.0
      cooldown_sec: 30.0
      max_flush_rate_hz: 20.0
      flush_topic: "/dc/flush"

    # Not armed: collects live as usual, and its Records leave incident_id NULL.
    memory:
      plugin: "dc_measurements/Memory"
      polling_interval: 1000
      group_key: "memory"
```

A runnable end-to-end version of this — two Measurements, one armed, both landing in the same
Postgres table, with the `incident_id` column asserted by query — is
`tools/e2e/scripts/run_incident.sh`.
