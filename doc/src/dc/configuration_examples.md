# Configuration examples

Through minimal code examples, you will learn how to collect and send data with DC.

It will progressively present all features.

```admonish info
By here, you must have built the workspace following the [setup guide](./setup.md).
```

Copy the configuration and save it as a yaml file, and then run:

```bash
ros2 launch dc_bringup params_file:="my_file.yaml"
```

## Running the examples
### Example 1: Uptime to the console every second

```yaml
dc_bridge:                                    # Bridge (Shipper) node configuration
  ros__parameters:
    destinations: ["console"]                 # List of Destination names to enable
    console:                                  # Destination name, you choose
      type: console                           # Blessed Destination type, fixed
      receives: records
      inputs: ["/dc/measurement/uptime"]      # Same as topic_output in the uptime measurement in measurement_server

measurement_server:                           # Measurement node configuration
  ros__parameters:
    measurement_plugins: ["uptime"]           # List of measurement plugins names to enable
    uptime:                                   # Plugin name, you choose
      plugin: "dc_measurements/Uptime"        # Plugin class name, fixed
      topic_output: "/dc/measurement/uptime"  # Topic where data will be published
      tags: ["console"]                       # The Bridge will match this against a Destination name
```

### Example 2: Uptime to the console with ISO 8601 timestamps

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/uptime"]
      time_key: "date"                       # Field the normalized timestamp is written to
      time_format: "iso8601"                 # "double" (Unix epoch seconds) or "iso8601"

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
      tags: ["console"]
```

### Example 3: Uptime to the console only at start and 3 times

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/uptime"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
      tags: ["console"]
      init_max_measurements: 3               # Maximum records to collect
```

### Example 4: CPU and Memory to the console every 5 seconds forever

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/cpu", "/dc/measurement/memory"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["memory", "cpu"]
    memory:
      plugin: "dc_measurements/Memory"
      topic_output: "/dc/measurement/memory"
      polling_interval: 5000                  # Interval to which data is collected in milliseconds
      tags: ["console"]
    cpu:
      plugin: "dc_measurements/Cpu"
      topic_output: "/dc/measurement/cpu"
      polling_interval: 5000                  # Interval to which data is collected in milliseconds
      tags: ["console"]
```

### Example 5: CPU and Memory as a group to the console every 5 seconds forever

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/group/cpu_memory"]        # Group to create

group_server:                                 # Group server configuration
  ros__parameters:
    groups: ["cpu_memory"]
    cpu_memory:
      inputs: ["/dc/measurement/cpu", "/dc/measurement/memory"] # Topics which are subscribed
      output: "/dc/group/cpu_memory"          # Topic where result will be published
      sync_delay: 5.0                         # How long to queue up messages before passing them through.
      group_key: "cpu_memory"
      tags: ["console"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["memory", "cpu"]
    memory:
      plugin: "dc_measurements/Memory"
      topic_output: "/dc/measurement/memory"
      polling_interval: 5000
      tags: ["console"]
    cpu:
      plugin: "dc_measurements/Cpu"
      topic_output: "/dc/measurement/cpu"
      polling_interval: 5000
      tags: ["console"]
```

### Example 6: Custom ROS message to the console every 2 seconds forever

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/string_stamped"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["my_string_stamped"]
    my_string_stamped:
      plugin: "dc_measurements/StringStamped"           # Plugin that allow to publish from your nodes
      topic_output: "/dc/measurement/my_string_stamped" # Topic where data is republished with tags
      topic: "/hello-world"                             # Input topic where you are publishing
      tags: ["console"]
      polling_interval: 2000
      enable_validator: false                           # By default, StringStamped message does not have a JSON schema since it uses custom input data
```

You will then need in another terminal to publish data on the input topic (`/hello-world`)

```bash
ros2 topic pub -r 1 /hello-world dc_interfaces/msg/StringStamped  "{data: '{\"hello\":\"world\"}'}"
```

### Example 7: Custom ROS message to the console every time it is published

```yaml
dc_bridge:
  ros__parameters:
    destinations: ["console"]
    console:
      type: console
      receives: records
      inputs: ["/dc/measurement/string_stamped"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["my_string_stamped"]
    my_string_stamped:
      plugin: "dc_measurements/StringStamped"
      topic_output: "/dc/measurement/my_string_stamped"
      topic: "/hello-world"
      tags: ["console"]
      enable_validator: false
      timer_based: false                                 # Get all data published on the input topic. Ignores polling_interval
```

Now that you know how it works, you can set your own measurements and destinations.
