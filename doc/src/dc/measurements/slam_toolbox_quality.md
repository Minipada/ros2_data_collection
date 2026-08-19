# slam_toolbox quality

## Description

Localization quality from two of [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)'s
native ROS 2 topics: `/pose` (`geometry_msgs/PoseWithCovarianceStamped`), polled on the same
interval as every other Measurement, for a `sample` Record carrying the pose and its covariance;
and `/slam_toolbox/loop_closure_event` (`slam_toolbox/LoopClosureEvent`) for a single-shot
`loop_closure` Record per occurrence.

`LoopClosureEvent` carries nothing but its own timestamp — slam_toolbox doesn't say which nodes
closed the loop or by how much, only that one happened. The Record reflects that: `loop_closure`
has no fields beyond `event`. The KPI value is entirely in the timing of these Records against
each other and against the samples: **loop closures per hour** is a rate over how often
`loop_closure` Records land, and **time since the last loop closure** — how long the map has gone
without a correction — is a localization-drift risk indicator on its own, the way a growing gap
between battery samples reads as data loss rather than a battery at 0 %.

Every Record carries an `event` field naming which of the two it is:

| `event`         | When                                  | Carries                                              |
| ---------------- | -------------------------------------- | ------------------------------------------------------- |
| `sample`         | Every polling interval, once `/pose` has published | `x`, `y`, `yaw`, and the diagonal covariance terms for each |
| `loop_closure`   | slam_toolbox reports one on `/slam_toolbox/loop_closure_event` | Nothing else — the occurrence is the fact |

The covariance terms are the diagonal of `PoseWithCovariance`'s 6x6 matrix at the indices for x, y
and yaw — the same terms AMCL/`robot_localization` dashboards already chart, and the ones that read
as confidence on their own axis without needing the off-diagonal correlations.

Until `/pose` publishes at all, no `sample` Record is emitted: a gap means no localization data,
not a robot at the origin. A loop closure is queued the moment it's reported and leaves on the
next poll, one Record per poll, so it travels the same path as every other Record (Conditions,
incident buffering, Group) — the same convention [Battery](./battery.md)'s charging-session
boundaries and [Intervention](./intervention.md)'s takeovers use.

```admonish info title="slam_toolbox is a runtime dependency, not a build one this plugin adds weight to"
`slam_toolbox/LoopClosureEvent` is defined inside the `slam_toolbox` package itself — it ships no
separate, lighter `_msgs` package the way nav2 does. This Measurement only makes sense on a robot
already running slam_toolbox for real SLAM, so depending on the real package (rather than working
around it with a raw/generic subscription) is the plain choice: the dependency is already present
wherever this Measurement is.
```

## Parameters

| Parameter               | Description                                                                 | Type | Default                              |
| ------------------------ | ----------------------------------------------------------------------------- | ---- | --------------------------------------- |
| **pose_topic**           | Topic (`geometry_msgs/PoseWithCovarianceStamped`) to read localization pose from | str  | "/pose" (Optional)                    |
| **loop_closure_topic**   | Topic (`slam_toolbox/LoopClosureEvent`) slam_toolbox reports loop closures on | str  | "/slam_toolbox/loop_closure_event" (Optional) |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "SlamToolboxQuality",
  "description": "Localization quality from slam_toolbox: a pose sample with covariance on the polling interval, plus a Record on every loop closure",
  "properties": {
    "event": { "type": "string", "enum": ["sample", "loop_closure"] },
    "x": { "type": "number" },
    "y": { "type": "number" },
    "yaw": { "type": "number" },
    "covariance_x": { "type": "number", "minimum": 0 },
    "covariance_y": { "type": "number", "minimum": 0 },
    "covariance_yaw": { "type": "number", "minimum": 0 }
  },
  "required": ["event"],
  "type": "object"
}
```

The full file (`plugins/measurements/json/slam_toolbox_quality.json`) also requires `x`, `y`,
`yaw` and all three covariance terms on a `sample` — `loop_closure` requires nothing beyond
`event`.

## Measurement configuration

```yaml
...
slam_quality:
  plugin: "dc_measurements/SlamToolboxQuality"
  topic_output: "/dc/measurement/slam_quality"
  polling_interval: 1000
  pose_topic: "/pose"
  loop_closure_topic: "/slam_toolbox/loop_closure_event"
```

Example Record data, one sample:

```json
{
  "event": "sample",
  "x": 1.42,
  "y": -0.63,
  "yaw": 0.71,
  "covariance_x": 0.008,
  "covariance_y": 0.011,
  "covariance_yaw": 0.004
}
```

A loop closure:

```json
{
  "event": "loop_closure"
}
```

## KPIs

`tools/infrastructure/sql/kpi_views.sql` defines `dc_kpi_loop_closure_rate()` (loop closures per
hour and seconds since the last one, over an arbitrary window) and `dc_kpi_loop_closures_1h` (the
same count bucketed hourly for charting) over this Measurement's `loop_closure` Records, following
the same convention as `dc_kpi_intervention_rate()`. The demo Grafana KPI dashboard charts loop
closures per hour; the robot dashboard charts the covariance trend directly from `sample` Records.
