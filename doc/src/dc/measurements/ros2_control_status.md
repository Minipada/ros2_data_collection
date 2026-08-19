# ROS2 control status

## Description

Reports when a `ros2_control` controller or hardware component crosses into or out of the
`active` lifecycle state -- the state in which it is actually commanding or reading hardware.
Rather than polling `list_controllers`/`list_hardware_components`, Ros2ControlStatus subscribes to
the controller manager's own `~/activity` topic
(`controller_manager_msgs/ControllerManagerActivity`), republished with transient-local QoS on
every controller/hardware-component lifecycle change, and feeds each component's stream of
`(state, timestamp)` samples to its own `dc_common::StateTransitionDetector` -- one detector per
component, mirroring Fault's per-component detector map, so components activate and deactivate
independently.

A Record is produced only for the boundary Intervention already reports in the same shape: a
transition into `active` starts one (`"event": "start"`), and a transition out of `active` ends
one (`"event": "end"`). A transition between two non-`active` states (e.g. `unconfigured` to
`inactive` during startup) crosses no boundary and produces nothing.

Every Record carries the component type (`controller` or `hardware_component`), its name, the
state it left (`from_state`) and entered (`to_state`), and how long the previous state had been
held (`previous_state_duration_s`) -- on a start Record, how long the component was out of
`active` before this; on an end Record, how long it had just been `active`. `open` is `true` on a
start Record and `false` on an end Record, so a component still active when collection stops
simply never gets a matching end Record and cannot be averaged as a zero. Every Record carries a
`seq` that increments by one across every controller and hardware component, so a dropped Record
is a detectable gap.

## Parameters

| Parameter | Description                                       | Type | Default                          |
| --------- | -------------------------------------------------- | ---- | --------------------------------- |
| **topic** | Topic to subscribe to for controller manager activity | str  | "/controller_manager/activity" (Optional) |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Ros2ControlStatus",
  "properties": {
    "seq": { "type": "integer", "minimum": 1 },
    "component_type": { "type": "string", "enum": ["controller", "hardware_component"] },
    "component": { "type": "string" },
    "event": { "type": "string", "enum": ["start", "end"] },
    "from_state": { "type": "string", "enum": ["unconfigured", "inactive", "active", "finalized", "configuring", "cleaningup", "shuttingdown", "activating", "deactivating", "errorprocessing", "unknown"] },
    "to_state": { "type": "string", "enum": ["unconfigured", "inactive", "active", "finalized", "configuring", "cleaningup", "shuttingdown", "activating", "deactivating", "errorprocessing", "unknown"] },
    "previous_state_duration_s": { "type": "number", "minimum": 0 },
    "open": { "type": "boolean" }
  },
  "required": ["seq", "component_type", "component", "event", "from_state", "to_state", "previous_state_duration_s", "open"],
  "type": "object"
}
```

## Measurement configuration

```yaml
...
ros2_control_status:
  plugin: "dc_measurements/Ros2ControlStatus"
  topic_output: "/dc/measurement/ros2_control_status"
  group_key: "ros2_control_status"
  tags: ["console"]
  topic: "/controller_manager/activity"
```

Example Record data, a start:

```json
{
  "seq": 1,
  "component_type": "controller",
  "component": "diff_drive_controller",
  "event": "start",
  "from_state": "inactive",
  "to_state": "active",
  "previous_state_duration_s": 4.2,
  "open": true
}
```

and its end:

```json
{
  "seq": 2,
  "component_type": "controller",
  "component": "diff_drive_controller",
  "event": "end",
  "from_state": "active",
  "to_state": "inactive",
  "previous_state_duration_s": 612.9,
  "open": false
}
```
