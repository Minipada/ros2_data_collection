# Camera

## Description

Save camera image files: raw, rotated and/or inspected. Images can be inspected using different
detection modules (e.g. barcode/QR detection, in-process via [ZXing-C++](https://github.com/zxing-cpp/zxing-cpp))

```admonish info title="Keep the whole code in frame, and rotation is not a concern"
ZXing-C++ replaced the Python `zbar`/`pyzbar` service (#123) mainly to remove a per-frame service
round trip and the last Python runtime dependency in the detection path. Measured against
`dc_simulation`'s demo QR assets under camera-realistic degradations, both libraries were perfect
on straight-on views, dim lighting, downscale+blur and mild (20%) perspective skew, and
ZXing-C++ is far more robust to **perspective/tilt distortion** (100% vs. 0% for `zbar` at a 35%
skew) — the physically realistic failure mode for a robot camera viewing a code off-axis.

This page previously warned that ZXing-C++ could not decode QR codes rotated 30-45° in-plane.
**That was a measurement artifact, not a library limitation** (#297): the benchmark rotated the
demo texture inside its own 290x365 bounds, and since the printed label offsets the 210 px code
from the centre it turns about, a corner of the code — with a finder pattern in it — swings out
of frame between 30° and 75°. A code the frame cuts a finder pattern off is unreadable at any
angle by any decoder. With the code fully in frame, `libzxing` 2.2.1 (the exact version `rosdep`
resolves on Ubuntu Noble/Jazzy) decodes it at every in-plane angle from 0° to 90°;
`dc_measurements`' `test_barcode_rotation` asserts both halves of that.

So the guidance is about framing, not angle: keep the whole code, plus its quiet zone, inside the
image. `rotation_angle` below only re-orients the image by whole quarter turns — it neither
causes nor fixes this, since it never crops.
```

## Parameters

| Parameter                 | Description                                                                                             | Type                 | Default                              |
| ------------------------- | ------------------------------------------------------------------------------------------------------- | -------------------- | ------------------------------------ |
| **cam_name**              | Name to give to the camera                                                                              | str                  | N/A (mandatory)                      |
| **cam_topic**             | Topic from where camera data needs to be fetched                                                        | str                  | N/A (mandatory)                      |
| **camera_info_topic**     | Topic to read the camera intrinsics from, for pose estimation                                           | str                  | `camera_info` next to **cam_topic**  |
| **code_size**             | Physical side length of the detected codes, in meters. Mandatory when **estimate_pose** is true         | double               | 0.0                                  |
| **detection_modules**     | Detection modules to use                                                                                | list\[str\](barcode) | N/A (optional)                       |
| **draw_det_barcodes**     | Draw barcode detection on images                                                                        | bool                 | true                                 |
| **estimate_pose**         | Estimate the pose of each detected code and add it to the Record                                        | bool                 | false                                |
| **pose_frame**            | Frame to transform the estimated pose into. Empty means the camera optical frame                        | str                  | ""                                   |
| **rotation_angle**        | Rotate the image before inspecting it by this angle                                                     | int (90, 180, 270)   | 0                                    |
| **transform_timeout**     | How long to wait for the **pose_frame** transform, in seconds                                           | double               | 0.1                                  |
| **save_detections_img**   | Whether to save inspected image captured by the camera with detection shapes                            | bool                 | true                                 |
| **save_inspected_base64** | Whether to save inspected image captured by the camera with detection shapes as base64 string           | bool                 | false                                |
| **save_inspected_path**   | Path to save the inspected camera image. Expands environment variables and datetime format are expanded | str                  | "camera/inspected/%Y-%m-%dT%H:%M:%S" |
| **save_raw_base64**       | Whether to save raw image captured by the camera as base64 string                                       | bool                 | false                                |
| **save_raw_img**          | Whether to save raw image captured by the camera                                                        | bool                 | false                                |
| **save_raw_path**         | Path to save the raw camera image. Expands environment variables and datetime                           | str                  | "camera/raw/%Y-%m-%dT%H:%M:%S"       |
| **save_rotated_base64**   | Whether to save rotated image captured by the camera as base64 string                                   | bool                 | false                                |
| **save_rotated_img**      | Whether to save rotated image captured by the camera                                                    | bool                 | false                                |
| **save_rotated_path**     | Path to save the rotated camera image. Expands environment variables and datetime format are expanded   | str                  | "camera/rotated/%Y-%m-%dT%H:%M:%S"   |

## Code pose estimation

With `estimate_pose: true`, every detected code carries a `pose` alongside its bounding box.
The pose is solved from the code's four detected corners with
[`cv::solvePnP`](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) (`SOLVEPNP_IPPE_SQUARE`,
the solver for four coplanar corners of a square), which needs two things detection alone does
not: the code's physical side length (`code_size`, in meters) and the camera intrinsics, read
from `camera_info_topic`. It is off by default and costs nothing when off — no `camera_info`
subscription is created and Records are unchanged.

### Frame and convention

The pose is the **code's** pose, not the robot's — where the code is as seen from the robot.
By default it is expressed in the **camera optical frame** (the `frame_id` of the `camera_info`
message, following [REP 103](https://ros.org/reps/rep-0103.html): X right, Y down, Z forward
along the lens axis), so `z` is the depth of the code in front of the camera. The code's own
frame is centered on the code and uses those same axes — X right, Y down, Z into its printed
face — so a code seen square-on has the **identity orientation**, and `roll`/`pitch`/`yaw` read
as how far off square-on it was. (This is the ArUco/OpenCV marker frame turned 180° about X:
that convention puts Z out of the face towards the camera, which would make a square-on read a
180° roll.)

Set `pose_frame` to have the pose transformed into a robot frame (`base_link`, `map`, …) via TF
before it is written. Every Record says which frame it is in: the emitted `pose.frame_id` is the
frame actually used, so if the transform is unavailable within `transform_timeout` the pose is
still reported — in the camera optical frame, with a warning logged, rather than dropped.
`pose.distance` is the camera-to-code range in meters and is unaffected by `pose_frame`.

`rotation_angle` is handled: the corners are mapped back to raw-image coordinates before the
solve, so the intrinsics still describe the image they were calibrated on.

```yaml
camera:
  plugin: "dc_measurements/Camera"
  cam_topic: "/front_camera/image_raw"
  # camera_info_topic defaults to /front_camera/camera_info, next to cam_topic
  cam_name: my_camera_with_codes
  detection_modules: ["barcode"]
  estimate_pose: true
  code_size: 0.2       # meters, side length of the printed code
  pose_frame: "base_link"
  transform_timeout: 0.1
```

```json
{
  "camera_name": "my_camera_with_codes",
  "inspected": {
    "barcode": [
      {
        "data": "0001", "type": "QRCode",
        "top": 210, "left": 295, "width": 84, "height": 84,
        "pose": {
          "frame_id": "base_link",
          "x": 1.482, "y": 0.037, "z": 0.611,
          "roll": 0.0, "pitch": 0.0, "yaw": 3.139,
          "distance": 1.483
        }
      }
    ]
  }
}
```

```admonish warning title="A pose is only as good as code_size and the calibration"
The scale of the estimate comes entirely from `code_size`: a code declared 20 cm wide that is
really 10 cm reports every distance twice as far as it is. Likewise the intrinsics are taken as
published — if `camera_info` carries an uncalibrated or placeholder camera matrix, the pose is
wrong without being flagged. Codes seen nearly edge-on or only a few pixels wide are also
poorly conditioned; use `pose.distance` to filter those out downstream.

Two systematic biases are worth knowing about before treating a pose as a measurement rather
than a hint:

- **Corner convention.** A detector locates the code to within about one module, so range
  carries a bias of roughly one module width — a few percent for a low-version QR code. This
  is a bias, not noise: averaging repeated reads does not remove it.
- **Non-square codes.** `SOLVEPNP_IPPE_SQUARE` assumes the four corners bound a square. A
  stretched or rectangular code is solved to a compromise scale. `dc_simulation`'s own
  `qrcode_*` assets are exactly this case — a 290x365 texture over a 0.5 x 0.5 m face makes
  the printed code 0.362 m across and 0.292 m down — which is why the demo sets `code_size`
  to a mid-value of 0.325 and why the simulation check tolerates a metre of error rather
  than centimetres.
```

## Measurement node configuration
The remote paths are also saved in the JSON under *<measurement_name>.<destination>_img_paths.(raw|rotated|inspected)*. If images want to be sent to a self-hosted S3-compatible store such as [RustFS](https://rustfs.com/), add "rustfs" in *remote_keys*. This will add a remote path that can later be used in your API.

Note that this remote key is not included in the JSON schema, which only contains the local paths. If you want to enforce the schema with your custom remote key, you will need to write it and load it manually.

```yaml
...
camera:
  plugin: "dc_measurements/Camera"
  group_key: "camera_with_codes"
  topic_output: "/dc/measurement/camera_with_codes"
  polling_interval: 10000
  init_collect: true
  node_name: "dc_measurement_camera"
  cam_topic: "/camera_with_codes"
  cam_name: my_camera_with_codes
  enable_validator: false
  draw_det_barcodes: true
  save_raw_img: true
  save_rotated_img: false
  save_detections_img: true
  save_raw_path: "camera_with_codes/raw/%Y-%m-%dT%H-%M-%S"
  save_rotated_path: "camera_with_codes/rotated/%Y-%m-%dT%H-%M-%S"
  save_inspected_path: "camera_with_codes/inspected/%Y-%m-%dT%H-%M-%S"
  rotation_angle: 0
  detection_modules: ["barcode"]
  remote_prefixes: [""]
  remote_keys: ["rustfs"] # Will create paths for RustFS, does not send the file
```

### Destination (dc_bridge) configuration
Now that the path is set, it can be used to know where to send the image. The
Destination name (`rustfs`) must match the `remote_keys` entry above — the Uploader
matches a Record's `remote_paths` keys against `receives: files` Destination names (see
[Destinations](../destinations.md)):

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["rustfs", "pgsql"]
    rustfs:
      type: s3
      receives: files
      inputs: ["/dc/group/cameras"]
      endpoint: "http://127.0.0.1:9000"
      access_key_id: "XEYqG4ZcPY5jiq5i"
      secret_access_key: "ji011KCtI82ZeQS6UwsQAg8x9VR4lSaQ"
      force_path_style: true
      bucket: "mybucket"
    files:
      metadata_destination: "pgsql"  # a receives: records Destination for status rows
```

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Camera",
  "description": "Camera images with detected objects",
  "properties": {
    "camera_name": {
      "description": "Name of the camera",
      "type": "string"
    },
    "local_paths": {
      "description": "Paths of saved images",
      "type": "object",
      "items": {
        "$ref": "#/$defs/paths"
      }
    },
    "remote_paths": {
      "description": "Dictionary of paths where metadata and images will be remotely stored",
      "type": "object",
      "additionalProperties": {
        "type": "object",
        "items": {
          "$ref": "#/$defs/paths"
        }
      }
    },
    "inspected": {
      "description": "Inspected content of an image",
      "type": "object",
      "items": {
        "$ref": "#/$defs/inspected"
      }
    }
  },
  "$defs": {
    "paths": {
      "type": "object",
      "properties": {
        "raw": {
          "description": "Raw image",
          "type": "string"
        },
        "rotated": {
          "description": "Rotated image",
          "type": "string"
        },
        "inspected": {
          "description": "Inspected image",
          "type": "string"
        }
      }
    },
    "inspected": {
      "type": "object",
      "properties": {
        "barcode": {
          "description": "Barcode inspected data",
          "type": "array",
          "items": {
            "$ref": "#/$defs/barcode"
          }
        }
      }
    },
    "barcode": {
      "type": "object",
      "properties": {
        "data": {
          "description": "Barcode data",
          "type": "string"
        },
        "height": {
          "description": "Barcode height",
          "type": "integer"
        },
        "width": {
          "description": "Barcode width",
          "type": "integer"
        },
        "top": {
          "description": "Barcode top position",
          "type": "integer"
        },
        "left": {
          "description": "Barcode left position",
          "type": "integer"
        },
        "type": {
          "description": "Barcode type",
          "type": "string"
        },
        "pose": {
          "description": "Pose of the code, present only when estimate_pose is enabled",
          "$ref": "#/$defs/pose"
        }
      }
    },
    "pose": {
      "type": "object",
      "properties": {
        "frame_id": {
          "description": "Frame the pose is expressed in",
          "type": "string"
        },
        "x": {
          "description": "Code position along the frame's X axis, in meters",
          "type": "number"
        },
        "y": {
          "description": "Code position along the frame's Y axis, in meters",
          "type": "number"
        },
        "z": {
          "description": "Code position along the frame's Z axis, in meters",
          "type": "number"
        },
        "roll": {
          "description": "Code orientation about the frame's X axis, in radians",
          "type": "number"
        },
        "pitch": {
          "description": "Code orientation about the frame's Y axis, in radians",
          "type": "number"
        },
        "yaw": {
          "description": "Code orientation about the frame's Z axis, in radians",
          "type": "number"
        },
        "distance": {
          "description": "Straight-line distance from the camera to the code, in meters",
          "type": "number"
        }
      }
    }
  },
  "type": "object"
}
```
