# Triggers

## Description

A **Trigger** is distinct from a **Condition**: a Condition gates whether a Measurement's
Records are collected right now, while a Trigger fires a one-shot signal that downstream
Measurements react to — e.g. releasing a pre-event buffer of recent data (later slices of
[#282](https://github.com/Minipada/ros2_data_collection/issues/282) wire that reaction up;
this package only builds the signal itself).

The `trigger_broadcast_node` loads the Condition plugins named in `condition_plugins` (the
same Condition plugins Measurements use — see [Conditions](./conditions.md)) plus one
Trigger plugin, polls the Trigger on a timer, and publishes a
`dc_interfaces/msg/FlushEvent` on a configurable topic (`/dc/flush` by default) each time
it fires. The node mints a fresh `incident_id` (a UUID) for every firing; nothing else
generates one — every subscriber shares one ID per incident.

```admonish note title="Only self-updating Conditions are meaningful here"
A Measurement passes its own freshly-collected Record to each Condition it consults, so
plugins like `BoolEqual` read a field out of that Record. A Trigger has no Record of its
own: it only makes sense to compose Conditions that maintain their own state from a
subscription, such as [Moving](./conditions/moving.md).
```

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
