# Camera

## Description

Save camera image files: raw, rotated and/or inspected. Images can be inspected using different
detection modules (e.g. barcode/QR detection, in-process via [ZXing-C++](https://github.com/zxing-cpp/zxing-cpp))

```admonish warning title="Barcode/QR detection accuracy is not uniformly better than the legacy zbar path"
ZXing-C++ replaced the Python `zbar`/`pyzbar` service (#123) mainly to remove a per-frame service
round trip and the last Python runtime dependency in the detection path, not because it is a
strictly more accurate decoder. Measured against `dc_simulation`'s demo QR assets under
camera-realistic degradations, both libraries were perfect on straight-on views, rotation up to
20°, dim lighting, downscale+blur, and mild (20%) perspective skew, but diverge outside that:

- **ZXing-C++ is far more robust to perspective/tilt distortion** (100% vs. 0% for `zbar` at a
  35% perspective skew) — the more physically realistic failure mode for a robot camera viewing
  a code off-axis.
- **`zbar` is more robust to pure in-plane rotation beyond ~20-25°.** ZXing-C++ (confirmed
  against the exact `libzxing-dev` 2.2.1 that ships via `rosdep` on Ubuntu Noble/Jazzy, not just
  the newer PyPI binding used for the initial comparison) fails to decode some QR codes rotated
  30-45° in-plane, misreporting a spurious `MicroQRCode`/`ChecksumError` instead of the real
  `QRCode`. This was root-caused, not just observed: it reproduces with nearest-neighbor
  rotation (rules out interpolation blur), with 3x upsampling (rules out resolution/module
  size), and across every `Binarizer` option (`LocalAverage`/`GlobalHistogram`/
  `FixedThreshold`) and with/without `tryDownscale` — a genuine finder-pattern-search limitation
  of this ZXing-C++ version for codes at these specific angles, not a tunable option.

If your deployment expects codes viewed at a steep in-plane rotation (not camera tilt) — e.g. a
QR code that itself may be mounted rotated relative to the camera's up axis — verify detection
empirically for your case; see #123's PR discussion for the full before/after methodology and
results.
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
  the printed code 0.362 m across and 0.288 m down — which is why the demo sets `code_size`
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
