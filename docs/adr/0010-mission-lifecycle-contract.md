# The Mission Measurement's lifecycle contract: nav2 adapter scope and Record shape

ROS has no standard interface for mission or task lifecycle. Issue #305 asked two
questions before any Mission Measurement could be written: how far a built-in nav2
adapter should go inferring "mission" from action goal status, and what an escape-hatch
message for stacks with their own mission executive should look like. This ADR records
that decision so #387 (the nav2 adapter) and later adapters (Open-RMF, a mission
executive) can implement against it without re-litigating scope.

**Decision: the nav2 adapter reports exactly one `NavigateToPose` goal's lifecycle, and
every adapter — nav2 or otherwise — emits the same `StringStamped`/JSON Record shape
used by every other Measurement, not a new typed `dc_interfaces` message.**

## Nav2 adapter scope

The nav2 adapter derives `mission_start`/`mission_end` purely from one `NavigateToPose`
goal: acceptance, terminal status, and result. It deliberately does not infer anything
above that:

- A "mission" that spans more than one nav2 goal (a multi-waypoint run coordinated by an
  external mission executive) is out of scope — nav2 has no concept of it, so DC will
  not guess at grouping goals into one mission.
- A mission sourced from a stack other than nav2 (Open-RMF, VDA5050, a custom mission
  executive) is out of scope for this adapter. `mission_id` and `mission_type` exist on
  the Record specifically so a future adapter can supply its own values without a schema
  change or reopening this decision.
- `NavigateThroughPoses` and `FollowWaypoints` are separate nav2 action interfaces with
  their own goal/feedback/result shapes; they get their own adapters, not a generalized
  one, tracked as issues blocked by #387.

## The escape-hatch: no new message, reuse the existing Record contract

Every Measurement in this codebase (`battery`, `intervention`, `fault`) publishes
`dc_interfaces::msg::StringStamped` carrying a JSON payload validated against a
`dc_measurements` JSON Schema — there is no precedent for a typed `dc_interfaces`
message per Measurement. The Mission Measurement does not introduce the first
exception: a stack with its own mission executive is a new Measurement plugin (or a
future generic "external mission" plugin) publishing the same Record shape below with
its own `mission_id`/`mission_type`, not a new `dc_interfaces` message type.

## Record shape

- `event`: `"mission_start"` or `"mission_end"`, required — matching `battery`'s
  `charge_session_start`/`_end` convention (a named episode kind), not
  `intervention`/`fault`'s generic `start`/`end`.
- `mission_id`: string, required on both events. The nav2 adapter synthesises it from
  the goal UUID (`unique_identifier_msgs/UUID`, stringified). Typed as a string, not an
  incrementing integer like `battery`'s `session_id`, so a future adapter can supply an
  externally issued id (a WMS order number, an Open-RMF booking id) without a type
  change.
- `mission_type`: optional string. The nav2 adapter sets it to the nav2 action name
  (`"navigate_to_pose"`, `"navigate_through_poses"`, `"follow_waypoints"` for sibling
  adapters). The schema accepts any string so a non-nav2 source can supply a richer
  value later without a schema change.
- `sequence`: integer, monotonic, incremented per emitted Record from one Measurement
  instance (not per `mission_id`) — gap detection, mirroring `fault`'s global counter.
- `outcome`: required on `mission_end` only. Enum: `succeeded`, `failed`, `cancelled`,
  `aborted`. For the nav2 adapter: `GoalStatus.ABORTED` → `aborted`,
  `GoalStatus.CANCELED` → `cancelled`, and `failed` is driven by a documented
  application-level condition (non-zero `NavigateToPose::Result.error_code` on an
  otherwise-succeeded goal status) rather than being collapsed into `aborted`.
- `reason`: string, required on `mission_end` when `outcome` is `failed` or `aborted`.
  Free text — for the nav2 adapter, verbatim from `NavigateToPose::Result.error_msg`.
  No DC-invented coded/structured reason table.
- `error_code`: integer, required alongside `reason`. For the nav2 adapter, verbatim
  from `NavigateToPose::Result.error_code`.
- `duration_sec`: number, required on `mission_end` only.
- `recoveries`: integer, optional on `mission_end` — for the nav2 adapter, from
  `NavigateToPose::Feedback.number_of_recoveries` at goal completion.
- No `update_id`/order-update field: a VDA5050-specific mid-mission-amendment concept
  with no evidence DC needs it; adding it later is a non-breaking schema change.

## A mission still running at shutdown

There is no explicit "open" field on the Record. A mission still running when the
process stops simply never gets a `mission_end` Record — nothing downstream can average
an unclosed interval as zero-duration, because there is nothing to average. Deriving an
"open"/still-running signal at query time (the way `dc_kpi_intervention_rate` derives
`open_interventions`) is the concern of the mission-success-rate view deferred from
#363, not this Record schema.

## Consequences

- #387 (the nav2 `NavigateToPose` adapter) and its siblings for `NavigateThroughPoses`
  and `FollowWaypoints` implement against this Record shape and scope boundary without
  needing further design discussion.
- A future non-nav2 adapter (Open-RMF is the leading candidate; VDA5050 explicitly is
  not, per #305) targets the same `mission_start`/`mission_end` contract, supplying its
  own `mission_id`/`mission_type`, rather than requiring a new message type or a schema
  redesign.
- `dc_measurements/plugins/measurements/json/mission_nav2.json` (and any sibling
  adapter's schema) validates against this shape; downstream views (#363's deferred
  mission-success-rate panel) can rely on `outcome` and `sequence` having the same
  meaning across every adapter that targets this contract.

## Considered Options

- A new typed `dc_interfaces` message for mission lifecycle events — rejected: no
  existing Measurement publishes a typed message for structured data: introducing one
  here would be the first exception to a load-bearing convention, for no benefit over
  the JSON Record + schema every other Measurement already uses.
- An adapter that groups nav2 goals into multi-goal missions itself (e.g. by proximity
  in time) — rejected: nav2 has no concept of a multi-goal mission, so any grouping
  heuristic DC invented would be a guess it could not honestly stand behind; the
  `mission_id`/`mission_type` escape hatch defers that grouping to a system that
  actually has the information.
