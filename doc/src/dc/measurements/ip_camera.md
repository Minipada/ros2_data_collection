# Ip Camera

## Description

Records video in small segments (in case of a cut) and store it locally.
They are first stored in a temporary folder. Once the record (of e.g 10 seconds) is done, it is moved to another directory.

Compared to other plugins, the collect function only moves the files from the temporary location, it does not start the recording. It takes some time to establish connection, so we avoid doing this every time. Recording is started at initialization by an ffmpeg process and saved in [HLS format](https://www.wikiwand.com/en/HTTP_Live_Streaming).

## Parameters

| Parameter            | Description                                                                                                        | Type                                            | Default                    |
| -------------------- | ------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------- | -------------------------- |
| **input**            | Input url                                                                                                          | str                                             | N/A (Mandatory)            |
| **video**            | Enable video recording                                                                                             | bool                                            | true                       |
| **audio**            | Enable audio recording                                                                                             | bool                                            | false                      |
| **bitrate_video**    | Video bitrate                                                                                                      | str(`[0-9]+[kmKM]`)                             | "2M"                       |
| **bitrate_audio**    | Audio bitrate                                                                                                      | str(`[0-9]+[kmKM]`)                             | "192k"                     |
| **segment**          | Records by small segment, managed by ffmpeg                                                                        | bool                                            | true                       |
| **segment_time**     | Duration of a segment                                                                                              | int (>0)                                        | 10                         |
| **ffmpeg_log_level** | Ffmpeg log level                                                                                                   | str (See [doc](https://ffmpeg.org/ffmpeg.html)) | "info"                     |
| **ffmpeg_banner**    | Show ffmpeg banner in console                                                                                      | bool                                            | true                       |
| **save_path**        | Path used to save files with ffmpeg, UTC date is used                                                             | str                                             | "ffmpeg_%Y-%m-%dT%H:%M:%S" |

```admonish warning title="The global save_local_base_path default crashes this Measurement"
`onConfigure()` expands `%Y`/`%M`/`%D`/`%H` in the directory it creates
(`dc_util::expand_time(storage_dir_)`), but `collect()` later iterates `storage_dir_`
**unexpanded**, as a literal path — and the default global `save_local_base_path`
(`$HOME/ros2/data/%Y/%M/%D/%H`, see [Measurements](../measurements.md)) contains exactly
those placeholders. The first `collect()` call throws an uncaught
`std::filesystem::filesystem_error` ("cannot open directory: No such file or directory")
and takes down the whole `measurement_server` process, along with every other Measurement
it runs. This is a known issue, still unfixed. Set a `save_local_base_path` with no `%`
placeholders (e.g. `/var/lib/dc`) to avoid it; `save_path` itself is unaffected, since only
its parent directory is extracted into `storage_dir_`.
```

## Schema

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "Ip Camera",
    "description": "Local and remote path where the remote camera video is recorded",
    "properties": {
        "local_path": {
            "description": "Local video path",
            "type": "string"
        },
        "remote_path": {
            "description": "Remote video path",
            "type": "string"
        }
    },
    "type": "object"
}
```

## Configuration

```yaml
...
ip_camera:
  plugin: "dc_measurements/IpCamera"
  topic_output: "/dc/measurement/ip_camera"
  input: "rtsp://192.168.0.10:554/stream1"
  segment_time: 10
  save_path: "ip_camera/%Y-%m-%dT%H-%M-%S"
```

## Example output

Captured from a real run, `input` pointed at a real-time MPEG-TS/TCP test stream
(`save_local_base_path` overridden per the warning above):

```json
{
  "data_src": "ip_camera",
  "flattened": false,
  "local_path": "/root/dc_capture_out/ip_camera/ffmpeg_2026-09-06T00:02:30.ts",
  "name": "ip_camera",
  "nested": false,
  "remote_path": "/ip_camera/ffmpeg_2026-09-06T00:02:30.ts",
  "run_id": "169",
  "timestamp": "ffmpeg_2026-09-06T00:02:30"
}
```
