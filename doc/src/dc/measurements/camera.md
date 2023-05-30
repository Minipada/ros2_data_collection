# Camera

## Description

Save camera image files: raw, rotated and/or inspected. Images can be inspected using different services (e.g barcode detection)

## Parameters

| Parameter                 | Description                                                                                             | Type                 | Default                              |
| ------------------------- | ------------------------------------------------------------------------------------------------------- | -------------------- | ------------------------------------ |
| **cam_name**              | Name to give to the camera                                                                              | str                  | N/A (mandatory)                      |
| **cam_topic**             | Topic from where camera data needs to be fetched                                                        | str                  | N/A (mandatory)                      |
| **detection_modules**     | Detection modules to use                                                                                | list\[str\](barcode) | N/A (optional)                       |
| **draw_det_barcodes**     | Draw barcode detection on images                                                                        | bool                 | true                                 |
| **rotation_angle**        | Rotate the image before inspecting it by this angle                                                     | int (90, 180, 270)   | 0                                    |
| **save_detections_img**   | Whether to save inspected image captured by the camera with detection shapes                            | bool                 | true                                 |
| **save_inspected_base64** | Whether to save inspected image captured by the camera with detection shapes as base64 string           | bool                 | false                                |
| **save_inspected_path**   | Path to save the inspected camera image. Expands environment variables and datetime format are expanded | str                  | "camera/inspected/%Y-%m-%dT%H:%M:%S" |
| **save_raw_base64**       | Whether to save raw image captured by the camera as base64 string                                       | bool                 | false                                |
| **save_raw_img**          | Whether to save raw image captured by the camera                                                        | bool                 | false                                |
| **save_raw_path**         | Path to save the raw camera image. Expands environment variables and datetime                           | str                  | "camera/raw/%Y-%m-%dT%H:%M:%S"       |
| **save_rotated_base64**   | Whether to save rotated image captured by the camera as base64 string                                   | bool                 | false                                |
| **save_rotated_img**      | Whether to save rotated image captured by the camera                                                    | bool                 | false                                |
| **save_rotated_path**     | Path to save the rotated camera image. Expands environment variables and datetime format are expanded   | str                  | "camera/rotated/%Y-%m-%dT%H:%M:%S"   |

## Measurement node configuration
The remote paths are also saved in the JSON under *<measurement_name>.<destination>_img_paths.(raw|rotated|inspected)*. If images want to be sent to Minio, add "minio" in *remote_keys*. This will add a remote path that can later be used in your API.

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
  remote_keys: ["minio"] # Will create paths for Minio, does not send the file
```

### Destination node configuration
Now that the path is set, it can be used to know where to send the image:

```yaml
...
flb_minio:
  verbose_plugin: false
  time_format: "iso8601"
  plugin: "dc_destinations/FlbMinIO"
  inputs: ["/dc/group/cameras"]
  endpoint: 127.0.0.1:9000
  access_key_id: XEYqG4ZcPY5jiq5i
  secret_access_key: ji011KCtI82ZeQS6UwsQAg8x9VR4lSaQ
  use_ssl: false
  create_bucket: true
  bucket: "mybucket"
  src_fields:
    [
      "camera.local_img_paths.raw",
      "camera.local_img_paths.inspected"
    ]
  upload_fields: # Remote paths created by the measurement node configuration
    [
      "camera.minio_img_paths.raw",
      "camera.minio_img_paths.inspected"
    ]
...
```

## Schema

```json
{
  "$schema":"http://json-schema.org/draft-07/schema#",
  "title":"Camera",
  "description":"Camera images with detected objects",
  "properties":{
    "camera_name": {
      "description": "Name of the camera",
      "type": "string"
    },
    "local_img_paths":{
      "description":"Paths of saved images",
      "type":"object",
      "items":{
        "$ref":"#/$defs/local_img_paths"
      }
    },
    "inspected":{
      "description":"Inspected content of an image",
      "type":"object",
      "items":{
        "$ref":"#/$defs/inspected"
      }
    }
  },
  "$defs":{
    "local_img_paths":{
      "type":"object",
      "properties":{
        "raw":{
          "description":"Raw image",
          "type":"string"
        },
        "rotated":{
          "description":"Rotated image",
          "type":"string"
        },
        "inspected":{
          "description":"Inspected image",
          "type":"string"
        }
      }
    },
    "inspected":{
      "type":"object",
      "properties":{
        "barcode":{
          "description":"Barcode inspected data",
          "type":"array",
          "items":{
            "$ref":"#/$defs/barcode"
          }
        }
      }
    },
    "barcode":{
      "type":"object",
      "properties":{
        "data":{
          "description":"Barcode data",
          "type":"array",
          "items":{
            "type":"integer"
          }
        },
        "height":{
          "description":"Barcode height",
          "type":"integer"
        },
        "width":{
          "description":"Barcode width",
          "type":"integer"
        },
        "top":{
          "description":"Barcode top position",
          "type":"integer"
        },
        "left":{
          "description":"Barcode left position",
          "type":"integer"
        },
        "type":{
          "description":"Barcode type",
          "type":"string"
        }
      }
    }
  },
  "type":"object"
}
```
