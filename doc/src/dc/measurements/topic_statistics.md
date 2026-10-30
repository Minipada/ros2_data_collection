# Topic statistics

## Description
Subscribes to `/statistics` (`statistics_msgs/MetricsMessage`) and publishes one Record per
message, so the per-topic statistics the ROS graph itself reports reach Destinations like any
other Measurement. The two known producers of that message are ROS 2's built-in topic
statistics (enabled per subscription with `enable_topic_statistics`) and
[ros2_pulse](https://github.com/TanayK07/ros2_pulse)'s `pulse_bridge`; this Measurement is a
generic `MetricsMessage` consumer and never subscribes to the topic being measured.
Run `./tools/e2e/scripts/run_pulse.sh` to prove the ros2_pulse path end to end: the script
runs one Podman container where a preloaded publisher feeds `pulse_bridge` and this
Measurement, and fails unless a Record with `average` and `sample_count` arrives.

The message's `StatisticDataPoint[]` array is enum-coded, so it is flattened into the five named
fields (`average`, `minimum`, `maximum`, `stddev`, `sample_count`). Every field is present in
every Record: `null` means the producer did not measure that statistic in the window, either
because the point was absent or because it was NaN — the "not measured" marker of rclcpp's
built-in collector and of `pulse_bridge`'s `metric` layout. No rate is derived here: it is
downstream work, from `sample_count` over the window or `1/average`.

## Enabling the ros2_pulse producer

The robot's processes run preloaded, per [ros2_pulse's README](https://github.com/TanayK07/ros2_pulse):

```bash
export LD_PRELOAD=$(ros2 pkg prefix ros2_pulse)/lib/libros2_pulse.so
export ROS_TOPIC_STATS_OUTPUT_FILE=/tmp/pulse.log
export ROS_TOPIC_STATS_FORMAT=jsonl
export ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=5.0
ros2 launch my_robot bringup.launch.py
```

`ROS_TOPIC_STATS_FORMAT=jsonl` is required: the probe's default is text, and `pulse_bridge`
refuses text logs, so with the default the bridge publishes nothing. Point the bridge's
`files` or `glob` at the same file as `ROS_TOPIC_STATS_OUTPUT_FILE`, and this Measurement's
`topic` at the bridge's `topic` parameter (both default to `/statistics`).

The bridge's `metrics_source` parameter picks what lands in the Record's `metrics_source`
field: the default `topic` keeps the measured topic's name there; `metric` is rclcpp's
built-in layout, where it is `message_period` / `message_age` and the topic name is gone.

By default the probe reports only `average` and `sample_count`: `minimum`, `maximum` and
`stddev` carry values only when the probed process tracks gaps — `ROS_TOPIC_STATS_JITTER=1`,
or an expected-rate spec (`ROS_TOPIC_STATS_EXPECTED`) that asks for it. Otherwise those
fields are `null`.

## Parameters

| Parameter | Description                              | Type | Default                 |
| --------- | ---------------------------------------- | ---- | ----------------------- |
| **topic** | Topic to subscribe to for topic statistics | str  | "/statistics" (Optional) |

## Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "TopicStatistics",
  "description": "Per-topic statistics from a statistics_msgs/MetricsMessage, as reported on the /statistics topic",
  "properties": {
    "measurement_source_name": {
      "description": "Node, topic or process name being measured",
      "type": "string"
    },
    "metrics_source": {
      "description": "The measured metric, e.g. the measured topic's name or \"message_age\"",
      "type": "string"
    },
    "unit": {
      "description": "Unit of the statistics, e.g. seconds or ms; empty when the producer left it unset",
      "type": "string"
    },
    "window_start": {
      "description": "Start of the window the statistics cover",
      "properties": {
        "sec": {
          "description": "Seconds since epoch",
          "type": "integer"
        },
        "nanosec": {
          "description": "Nanosecond part of the timestamp",
          "type": "integer"
        }
      },
      "required": ["sec", "nanosec"],
      "type": "object"
    },
    "window_stop": {
      "description": "End of the window the statistics cover",
      "properties": {
        "sec": {
          "description": "Seconds since epoch",
          "type": "integer"
        },
        "nanosec": {
          "description": "Nanosecond part of the timestamp",
          "type": "integer"
        }
      },
      "required": ["sec", "nanosec"],
      "type": "object"
    },
    "average": {
      "description": "Mean over the window; null when the producer did not measure it",
      "type": ["number", "null"]
    },
    "minimum": {
      "description": "Smallest value observed in the window; null when the producer did not measure it",
      "type": ["number", "null"]
    },
    "maximum": {
      "description": "Largest value observed in the window; null when the producer did not measure it",
      "type": ["number", "null"]
    },
    "stddev": {
      "description": "Standard deviation over the window; null when the producer did not measure it",
      "type": ["number", "null"]
    },
    "sample_count": {
      "description": "Messages observed in the window; null when the producer did not report it",
      "type": ["integer", "null"]
    }
  },
  "required": ["measurement_source_name", "metrics_source", "window_start", "window_stop"],
  "type": "object"
}
```

## Configuration

```yaml
...
topic_statistics:
  plugin: "dc_measurements/TopicStatistics"
  topic_output: "/dc/measurement/topic_statistics"
```

## Example output

```json
{
  "measurement_source_name": "topic_statistics_test_node",
  "metrics_source": "/cmd_vel",
  "unit": "ms",
  "window_start": {"sec": 1700000000, "nanosec": 100},
  "window_stop": {"sec": 1700000010, "nanosec": 200},
  "average": 12.5,
  "minimum": 8.0,
  "maximum": 30.0,
  "stddev": 4.2,
  "sample_count": 5
}
```
