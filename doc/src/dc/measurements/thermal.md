# Thermal

## Description
Reports temperatures (CPU, GPU, board, ...) from the kernel's thermal sysfs interface
(`/sys/class/thermal/thermal_zone*/`), so overheating trends reach dashboards before they
become failures. Zones are auto-discovered by default; each zone's `type` string (e.g.
`x86_pkg_temp`, `cpu-thermal`) is used as the Record key, not its numeric index, since zone
numbering is platform-specific and differs between x86_64 and ARM targets. A missing or
unreadable `/sys/class/thermal` (e.g. a container without the host's thermal sysfs mounted)
degrades gracefully: activation still succeeds, and if no zone can currently be read the
Measurement publishes nothing that cycle (the same "silent, retry next poll" contract other
hardware-optional Measurements like Serial interface use) instead of failing or publishing an
empty Record.

## Parameters

| Parameter     | Description                                                                                              | Type        | Default                |
| ------------- | --------------------------------------------------------------------------------------------------------- | ----------- | ------------------------ |
| **base_path** | Directory to scan for `thermal_zone*` entries                                                              | str         | "/sys/class/thermal" (Optional) |
| **zones**     | Explicit list of zone directory names (e.g. `["thermal_zone0", "thermal_zone2"]`) to read instead of auto-discovering every `thermal_zone*` entry under **base_path** | list of str | [] (Optional)             |

## Schema

Zone type strings are the field names, so they can't be listed ahead of time — the schema
constrains their shape instead: at least one entry (the Measurement publishes nothing rather
than an empty Record), a non-empty type string as key, and a temperature in degrees Celsius
above absolute zero as value.

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Thermal",
    "description": "One entry per thermal zone read this cycle, keyed by the zone's type string",
    "propertyNames": {
        "description": "The zone's type string, e.g. x86_pkg_temp or cpu-thermal",
        "minLength": 1
    },
    "additionalProperties": {
        "description": "Temperature of the zone in degrees Celsius",
        "type": "number",
        "minimum": -273.15
    },
    "minProperties": 1,
    "type": "object"
}
```

## Measurement configuration

```yaml
...
thermal:
  plugin: "dc_measurements/Thermal"
  topic_output: "/dc/measurement/thermal"
  tags: ["console"]
```

Example Record data, on a host exposing a CPU package zone and a GPU zone:

```json
{
  "x86_pkg_temp": 52.0,
  "gpu-thermal": 61.5
}
```
