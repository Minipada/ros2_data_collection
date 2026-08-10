# `dc_util` owns parameter declaration; `nav2_util` stays a dependency

Before this decision, parameter declaration in `dc_measurements`/`dc_group` was three
patterns at once: most Measurement/Condition plugins called
`nav2_util::declare_parameter_if_not_declared` directly, duplicating the same
declare-then-`get_parameter`-then-fatal-on-failure boilerplate in each `onConfigure()`;
`measurement_server.cpp` mixed that with a handful of raw `declare_parameter` calls for
its own node-level parameters (which throw on redeclaration instead of being idempotent);
and a `dc_util::get_*_type_param()` helper family existed but was only used in ~20 of the
roughly 90 plugin-parameter call sites, wrapping `nav2_util::declare_parameter_if_not_declared`
for the rest without most callers going through it.

## Decision

- **`nav2_util` stays a dependency.** `MeasurementServer` already inherits
  `nav2_util::LifecycleNode` for the bond/lifecycle machinery `dc_lifecycle_manager`
  orchestrates — dropping `nav2_util` isn't on the table regardless of how parameter
  declaration is handled, so re-implementing `declare_parameter_if_not_declared`'s
  idempotent-declare logic inside `dc_util` would only add a second implementation of
  the same thing for no dependency-removal benefit.
- **`dc_util::get_*_type_param()` / `get_*_param()` become the single sanctioned way to
  declare a parameter in `dc_measurements`/`dc_group` C++ code.** Plugin authors call
  these, never `declare_parameter` or `nav2_util::declare_parameter_if_not_declared`
  directly (documented in `doc/src/dc/contributing.md`). Internally these helpers still
  call `nav2_util::declare_parameter_if_not_declared` — `dc_util/include/dc_util/node_utils.hpp`
  is now the *only* file in `dc_measurements`/`dc_group` allowed to reference `nav2_util`
  for parameter declaration — so the dependency is kept, but callers no longer see it or
  hand-roll its error handling.
- The helper family was extended to cover every parameter type actually declared across
  the plugins (`double`, and mandatory `vector<bool>`/`vector<int64_t>`/`vector<double>`
  were missing) plus a node-level (unprefixed) variant for `MeasurementServer`'s own
  parameters, so the raw `declare_parameter` calls in its constructor could move to the
  same idempotent pattern as everything else.
- **`dc_group` (Python/rclpy) keeps its existing plain `self.declare_parameter(...)`
  calls.** There is no Python equivalent of `nav2_util::declare_parameter_if_not_declared`
  in this codebase, and none is needed: every `GroupServer` parameter is declared exactly
  once, in `init_parameters()`, never re-entered — the idempotent-declare problem the C++
  helpers solve (multiple plugins/onConfigure() calls potentially racing to declare a
  shared namespace) doesn't exist on the group-server side. Introducing a
  declare-if-not-declared wrapper there would be solving a problem this file doesn't have.

## Consequences

- A new plugin parameter is one `dc_util::get_*_type_param()` call, not a
  declare-if-not-declared pair plus a manual fatal-on-missing check.
- Two pre-existing bugs surfaced while converting call sites — `bool_equal.cpp`'s `value_`
  field is `double` despite the parameter being declared `PARAMETER_BOOL`, and
  `distance_traveled.cpp` declares `transform_tolerance` but reads back the different,
  undeclared name `transform_timeout` — were deliberately left as direct
  `nav2_util::declare_parameter_if_not_declared`/`get_parameter` calls rather than folded
  into the new helpers, so as not to silently change behavior while unifying the
  declaration *pattern*. Both are noted inline and are follow-up work, not part of this
  change.
