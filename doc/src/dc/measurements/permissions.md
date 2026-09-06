# Permissions

## Description

Collect UID, GID, if a file or directory exists and its permissions (in rwx or integer format).

## Parameters

| Parameter  | Description                                                                       | Type            | Default         |
| ---------- | --------------------------------------------------------------------------------- | --------------- | --------------- |
| **format** | Format to collect permissions in                                                  | str(rwx or int) | "int"           |
| **path**   | Path to the file or directory to collect data from, support environment variables | str             | N/A (mandatory) |

## Schema

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

```admonish info
`permissions.cpp`'s `collect()` also adds `user` and `group` (the owning username/group
name from `getpwuid`/`getgrgid`) whenever the uid/gid resolves on the local system —
neither is in the schema above. They're absent, not empty, when the file is owned by an
id the container doesn't recognize (e.g. owned by the host, not the container).
```

## Configuration

```yaml
...
permission_home_dc:
  plugin: "dc_measurements/Permissions"
  topic_output: "/dc/measurement/permissions_home_dc"
  path: "$HOME/dc"
  format: "rwx"
```

## Example output

```json
{
  "exists": true,
  "flattened": false,
  "gid": 0,
  "group": "root",
  "name": "permission_home_dc",
  "nested": false,
  "permissions": "rwx------",
  "run_id": "169",
  "uid": 0,
  "user": "root"
}
```
