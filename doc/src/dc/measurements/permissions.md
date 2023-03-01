# Permissions

## Description

Collect UID, GID, if a file or directory exists and its permissions (in rwx or integer format).

## Parameters

| Parameter  | Description                                                                       | Type         | Default         |
| ---------- | --------------------------------------------------------------------------------- | ------------ | --------------- |
| **format** | Format to collect permissions in. RWX = 0, INT = 1                                | Enum(0 or 1) | 0               |
| **path**   | Path to the file or directory to collect data from, support environment variables | str          | N/A (mandatory) |

## Schemas

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Permissions",
    "description": "Permissions of a file/directory",
    "properties": {
        "uid": {
            "description": "File/directory User IDentifier",
            "type": "integer"
        },
        "gid": {
            "description": "File/directory Group IDentifier",
            "type": "integer"
        },
        "exists": {
            "description": "File/directory exists",
            "type": "boolean"
        },
        "permissions": {
            "description": "Permissions as rwx or integer",
            "type": "string"
        }
    },
    "type": "object"
}
```

## Measurement configuration

```yaml
...
permission_home_dc:
  plugin: "dc_measurements/Permissions"
  topic_output: "/dc/measurement/permissions_home_dc"
  tags: ["flb_stdout"]
  path: "$HOME/dc"
  format: 1
```
