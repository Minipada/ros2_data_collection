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
| **segment**          | Records by small segment, managed by ffmpeg                                                                        | bool                                            | bool                       |
| **segment_time**     | Duration of a segment                                                                                              | int (>0)                                        | 10                         |
| **ffmpeg_log_level** | Ffmpeg log level                                                                                                   | str (See [doc](https://ffmpeg.org/ffmpeg.html)) | "info"                     |
| **ffmpeg_banner**    | Show ffmpeg banner in console                                                                                      | bool                                            | true                       |
| **save_path**        | Path used to save files with ffmpeg, UTC date is used. There can't be ":" in this string, ffmpeg does not parse it | str                                             | "ffmpeg_%Y-%m-%dT%H:%M:%S" |

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
