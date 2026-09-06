# Overview

## Description
A condition enables or disables one or multiple measurements to be published and thus collected. We could for example enable collecting camera images only when a robot is stopped.

Each condition is enabled or disabled through a pluginlib plugin. It has these configuration parameters.

```admonish note title="Conditions only see forward in time"
A Condition gates collection for as long as its predicate holds, giving you data from the
moment it became true onward. For what happened *before* an event, use a
[Trigger](./triggers.md) instead — built from the same Condition plugins, it fires once on
the false→true edge and releases a window a Measurement had already buffered.
```

## Wiring a Condition to a Measurement

Configuring a Condition is a two-step wire-up, both under `measurement_server`:

1. Declare it in `condition_plugins` and give it its own named block, exactly like a
   Measurement — `plugin` names the pluginlib class, and any other keys are that
   Condition's own [parameters](#available-plugins).
2. Reference that name from a Measurement's `if_all_conditions`, `if_any_conditions`,
   `if_none_conditions`, or `gate_condition` (see [Measurements](./measurements.md#plugin-parameters)).
   The Measurement is what names the Condition — nothing on the Condition side says which
   Measurement it gates, so one Condition can gate several Measurements at once.

The camera-only-when-stopped example above, taken from a real demo
(`dc_demos/params/qrcodes_stdout.yaml`):

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["moving"]
    moving:
      plugin: "dc_conditions/Moving"
    right_camera:
      plugin: "dc_measurements/Camera"
      if_none_conditions: ["moving"] # collect only while NOT moving
      topic_output: "/dc/measurement/right_camera"
      cam_topic: "/right_intel_realsense_r200_depth/image_raw"
```

## Available plugins:

| Name                                                     | Description                                      |
| -------------------------------------------------------- | ------------------------------------------------ |
| [Robot moving](./conditions/moving.md)                   | Robot is moving                                  |
| [Bool equal](./conditions/bool_equal.md)                 | Value of a boolean key is equal to               |
| [Double equal](./conditions/double_equal.md)             | Value of a double key is equal to                |
| [Double inferior](./conditions/double_inferior.md)       | Value of a double key is inferior to             |
| [Double superior](./conditions/double_superior.md)       | Value of a double key is superior to             |
| [Exist](./conditions/exist.md)                           | Key exists                                       |
| [Integer equal](./conditions/integer_equal.md)           | Value of an integer key is equal to              |
| [Integer inferior](./conditions/integer_inferior.md)     | Value of an integer key is inferior to           |
| [Integer superior](./conditions/integer_superior.md)     | Value of an integer key is superior to           |
| [List bool equal](./conditions/list_bool_equal.md)       | Value of a list of boolean key is equal to       |
| [List double equal](./conditions/list_double_equal.md)   | Value of a list of double key is equal to        |
| [List integer equal](./conditions/list_integer_equal.md) | Value of a list of integer key is equal to       |
| [List string equal](./conditions/list_string_equal.md)   | Value of a list of string key is equal to        |
| [Same as previous](./conditions/same_as_previous.md)     | Value of the key is the same as the previous one |
| [String match](./conditions/string_match.md)             | Value of a string key matches the regex of       |
