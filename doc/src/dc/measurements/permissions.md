# Permissions

## Description

Collect UID, GID, if a file or directory exists and its permissions (in rwx or integer format).

## Parameters

| Parameter  | Description                                        | Type         | Default |
| ---------- | -------------------------------------------------- | ------------ | ------- |
| **path**   | Path to the file or directory to collect data from | str          | ""      |
| **env**    | Environment from which to get the path from        | str          | ""      |
| **format** | Format to collect permissions in. RWX = 0, INT = 1 | Enum(0 or 1) | 0       |

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
