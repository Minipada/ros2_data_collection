# Serial interface

## Description
Reads line-delimited data off a configurable serial port (baud rate configurable) and parses
each line into named fields, publishing them as a Record — for custom robot sensors/boards that
talk over a UART/USB-serial link and never reach a ROS topic on their own.

The port is opened lazily, on the first poll after activation, and never on `onConfigure()` — an
unplugged or not-yet-connected device does not fail activation. If the port disconnects (read
error or EOF, e.g. the USB adapter is unplugged), the Measurement logs a warning, closes the
file descriptor, and keeps polling; the next poll after the device reappears reopens and resumes
normally, with no operator action needed and no busy-looping in between (opening only happens
once per `polling_interval` tick).

Only line-delimited framing (`\n`, with an optional trailing `\r` stripped) is implemented today;
`framing` is still a configuration knob for future framing modes. Only the most recently completed
line in a given poll is parsed — if several lines arrive within one `polling_interval`, earlier
ones are dropped, the same lossy-between-polls behavior other subscription/poll-driven
Measurements (e.g. `cmd_vel`, `diagnostics`) already have.

Two parsing modes are supported, both producing the same `fields` object shape:
- `delimiter`: splits the line on `delimiter` and assigns tokens to `fields` in order.
- `regex`: matches the line against `regex` and assigns capture groups to `fields` in order.
  `std::regex`/ECMAScript has no native named-capture-group syntax, so "named groups" here means
  pairing each positional capture group with a name from `fields`, in capture order.

If the token/capture count doesn't match the configured `fields` count, a warning is logged and
whatever fields do line up are still published — a malformed line degrades rather than drops.

## Parameters

| Parameter        | Description                                                                              | Type      | Default       |
| ----------------- | ------------------------------------------------------------------------------------------ | --------- | -------------- |
| **port**          | Serial device path (e.g. `/dev/ttyUSB0`)                                                   | str       | "" (required)  |
| **baud_rate**     | Baud rate: one of 1200/2400/4800/9600/19200/38400/57600/115200/230400                      | int       | 9600           |
| **framing**       | Line framing mode; only `"line"` is implemented                                            | str       | "line"         |
| **parsing_type**  | `"delimiter"` or `"regex"`                                                                 | str       | "delimiter"    |
| **delimiter**     | Delimiter string used when `parsing_type: delimiter`                                       | str       | ","            |
| **regex**         | ECMAScript regex (with capture groups) used when `parsing_type: regex`                     | str       | ""             |
| **fields**        | Ordered field names paired with delimiter tokens or regex capture groups                   | list[str] | [] (Optional)  |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "SerialInterface",
  "description": "A line parsed off a configured serial port",
  "properties": {
    "raw": {
      "description": "The raw line read from the serial port, with any trailing CR/LF stripped",
      "type": "string"
    },
    "fields": {
      "description": "Named fields extracted from 'raw' per the configured parsing (delimiter split or regex capture groups)",
      "type": "object"
    }
  },
  "required": ["raw", "fields"],
  "type": "object"
}
```

## Measurement configuration

Delimiter split, e.g. a board emitting `23.5,60\n` (temperature, humidity):

```yaml
...
serial_sensor:
  plugin: "dc_measurements/SerialInterface"
  topic_output: "/dc/measurement/serial_sensor"
  tags: ["console"]
  port: "/dev/ttyUSB0"
  baud_rate: 9600
  parsing_type: "delimiter"
  delimiter: ","
  fields: ["temperature", "humidity"]
```

Regex capture groups, e.g. a board emitting `T:23.5 H:60\n`:

```yaml
...
serial_sensor:
  plugin: "dc_measurements/SerialInterface"
  topic_output: "/dc/measurement/serial_sensor"
  tags: ["console"]
  port: "/dev/ttyUSB0"
  baud_rate: 9600
  parsing_type: "regex"
  regex: "^T:(\\d+\\.\\d+) H:(\\d+)$"
  fields: ["temperature", "humidity"]
```

## Testing without hardware

The gtest suite (`test/test_measurement_serial_interface.cpp`) verifies this Measurement against
a virtual serial pair created with `socat`, no hardware required:

```bash
socat -d -d pty,raw,echo=0,link=/tmp/dc_serial_dev pty,raw,echo=0,link=/tmp/dc_serial_peer
```

The Measurement is pointed at `/tmp/dc_serial_dev`; the test writes fixture lines to
`/tmp/dc_serial_peer`. Killing and restarting `socat` against the same `link=` paths simulates an
unplug/replug cycle and exercises the reconnect path.
