# String stamped

## Description

Collect generic data from a topic publishing a StringStamped message and republish it. It allows to fetch data from rclc, rclpy and your custom ROS 2 nodes that don't have use a plugin.

## Parameters

| Parameter       | Description                                                                                      | Type | Default         |
| --------------- | ------------------------------------------------------------------------------------------------ | ---- | --------------- |
| **timer_based** | If true, collect data at interval and if false collect every record and ignores polling_interval | bool | true            |
| **topic**       | Topic to get data from                                                                           | str  | N/A (mandatory) |


## Schema

Given that the data is customized here, there is no default schema.

## Configuration

```yaml
...
my_data:
  plugin: "dc_measurements/StringStamped"
  topic_output: "/dc/measurement/my_data"
  topic: "/hello_world"
  timer_based: true
  enable_validator: false # no schema exists for custom, per-topic data -- see Schema above
```

## Example output

```json
{
  "flattened": false,
  "name": "string_stamped",
  "nested": false,
  "run_id": "169",
  "temperature": 21.5
}
```
