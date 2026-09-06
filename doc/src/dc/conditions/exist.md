# Exist

## Description

Compare JSON key value to the value passed in parameter and returns true if exists.

## Parameters

| Parameter | Description                                                            | Type | Default         |
| --------- | ---------------------------------------------------------------------- | ---- | --------------- |
| **key**   | JSON key where value is located, separate nested dictionary with **/** | str  | N/A (Mandatory) |

## Configuration

Only forward a [Camera](../measurements/camera.md) Record once it actually inspected
something (`dc_demos/params/qrcodes_minio_pgsql.yaml`):

```yaml
measurement_server:
  ros__parameters:
    condition_plugins: ["moving", "inspected_exists"]
    inspected_exists:
      plugin: "dc_conditions/Exist"
      key: "inspected"
    right_camera:
      plugin: "dc_measurements/Camera"
      if_none_conditions: ["moving"]
      if_all_conditions: ["inspected_exists"]
      topic_output: "/dc/measurement/right_camera"
```
