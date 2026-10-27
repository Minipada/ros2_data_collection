# Topic statistics

## Description
Subscribes to `/statistics` (`statistics_msgs/MetricsMessage`) and publishes one Record per
message, so the per-topic statistics the ROS graph itself reports reach Destinations like any
other Measurement. The two known producers of that message are ROS 2's built-in topic
statistics (enabled per subscription with `enable_topic_statistics`) and
[ros2_pulse](https://github.com/TanayK07/ros2_pulse)'s `pulse_bridge`; this Measurement is a
generic `MetricsMessage` consumer and never subscribes to the topic being measured.

The message's `StatisticDataPoint[]` array is enum-coded, so it is flattened into named fields
(`average`, `minimum`, `maximum`, `stddev`, `sample_count`). A statistic the producer did not
include stays out of the Record. No rate is derived here: it is downstream work, from
`sample_count` over the window or `1/average`.

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
      "description": "Mean over the window",
      "type": "number"
    },
    "minimum": {
      "description": "Smallest value observed in the window",
      "type": "number"
    },
    "maximum": {
      "description": "Largest value observed in the window",
      "type": "number"
    },
    "stddev": {
      "description": "Standard deviation over the window",
      "type": "number"
    },
    "sample_count": {
      "description": "Messages observed in the window",
      "type": "integer"
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
