# Moving

## Description

Use a [hysteresis](https://www.wikiwand.com/en/Hysteresis) on robot position to know whether the robot is moving.

## Parameters

| Parameter            | Description                                                  | Type  | Default |
| -------------------- | ------------------------------------------------------------ | ----- | ------- |
| **odom_topic**       | Topic from where odom is used to know if the robot is moving | str   | "/odom" |
| **speed_threshold**  | Speed threshold used in the hysteresis                       | float | 0.2     |
| **count_limit**      | Counter to know if the robot is moving                       | int   | 8       |
| **count_hysteresis** | Hysteresis counter                                           | int   | 5       |

## Configuration

Only record camera images while the robot is stationary
(`dc_demos/params/qrcodes_stdout.yaml`):

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["moving"]
    moving:
      plugin: "dc_conditions/Moving"
    right_camera:
      plugin: "dc_measurements/Camera"
      if_none_conditions: ["moving"]
      topic_output: "/dc/measurement/right_camera"
      cam_topic: "/right_intel_realsense_r200_depth/image_raw"
```
