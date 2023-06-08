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
### Example 1: Uptime to Fluent Bit Stdout every second and flush data every second

```yaml
destination_server:                           # Destination node configuration
  ros__parameters:
    destination_plugins: ["flb_stdout"]       # List of destination plugins names to enable
    flb_stdout:                               # Plugin name, you choose
      plugin: "dc_destinations/FlbStdout"     # Plugin class name, fixed
      inputs: ["/dc/measurement/uptime"]      # Same as topic_output in the uptime measurement in measurement_server

measurement_server:                           # Measurement node configuration
  ros__parameters:
    measurement_plugins: ["uptime"]           # List of measurement plugins names to enable
    uptime:                                   # Plugin name, you choose
      plugin: "dc_measurements/Uptime"        # Plugin class name, fixed
      topic_output: "/dc/measurement/uptime"  # Topic where data will be published
      tags: ["flb_stdout"]                    # Fluent Bit will match this in the destination server
```

### Example 2: Uptime to Fluent Bit Stdout every second and flush data every 3 seconds

```yaml
destination_server:
  ros__parameters:
    flb:
      flush: 3                               # Interval to flush output (seconds)
    destination_plugins: ["flb_stdout"]
    flb_stdout:
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/measurement/uptime"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
      tags: ["flb_stdout"]
```

### Example 3: Uptime to Fluent Bit Stdout only at start and 3 times

```yaml
destination_server:
  ros__parameters:
    destination_plugins: ["flb_stdout"]
    flb_stdout:
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/measurement/uptime"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["uptime"]
    uptime:
      plugin: "dc_measurements/Uptime"
      topic_output: "/dc/measurement/uptime"
      tags: ["flb_stdout"]
      init_max_measurements: 3               # Maximum records to collect
```

### Example 4: CPU and Memory to Fluent Bit Stdout every 5 seconds forever

```yaml
destination_server:
  ros__parameters:
    destination_plugins: ["flb_stdout"]
    flb_stdout:
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/measurement/cpu", "/dc/measurement/memory"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["memory", "cpu"]
    memory:
      plugin: "dc_measurements/Memory"
      topic_output: "/dc/measurement/memory"
      polling_interval: 5000                  # Interval to which data is collected in milliseconds
      tags: ["flb_stdout"]
    cpu:
      plugin: "dc_measurements/Cpu"
      topic_output: "/dc/measurement/cpu"
      polling_interval: 5000                  # Interval to which data is collected in milliseconds
      tags: ["flb_stdout"]
```

### Example 5: CPU and Memory as a group to Fluent Bit Stdout every 5 seconds forever

```yaml
destination_server:
  ros__parameters:
    destination_plugins: ["flb_stdout"]
    flb_stdout:
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/group/memory_cpu"]        # Group to create

group_server:                                 # Group server configuration
  ros__parameters:
    groups: ["cpu_memory"]
    cpu_memory:
      inputs: ["/dc/measurement/cpu", "/dc/measurement/memory"] # Topics which are subscribed
      output: "/dc/group/cpu_memory"          # Topic where result will be published
      sync_delay: 5.0                         # How long to queue up messages before passing them through.
      group_key: "cpu_memory"
      tags: ["flb_stdout"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["memory", "cpu"]
    memory:
      plugin: "dc_measurements/Memory"
      topic_output: "/dc/measurement/memory"
      polling_interval: 5000
      tags: ["flb_stdout"]
    cpu:
      plugin: "dc_measurements/Cpu"
      topic_output: "/dc/measurement/cpu"
      polling_interval: 5000
      tags: ["flb_stdout"]
```

### Example 6: Custom ROS message to Stdout every 2 seconds forever

```yaml
destination_server:
  ros__parameters:
    destination_plugins: ["flb_stdout"]
    flb_stdout:
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/measurement/string_stamped"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["my_string_stamped"]
    my_string_stamped:
      plugin: "dc_measurements/StringStamped"           # Plugin that allow to publish from your nodes
      topic_output: "/dc/measurement/my_string_stamped" # Topic where data is republished with tags
      topic: "/hello-world"                             # Input topic where you are publishing
      tags: ["flb_stdout"]
      polling_interval: 2000
      enable_validator: false                           # By default, StringStamped message does not have a JSON schema since it uses custom input data
```

You will then need in another terminal to publish data on the input topic (`/hello-world`)

```bash
ros2 topic pub -r 1 /hello-world dc_interfaces/msg/StringStamped  "{data: '{\"hello\":\"world\"}'}"
```

### Example 7: Custom ROS message to Stdout every time it is published

```yaml
destination_server:
  ros__parameters:
    destination_plugins: ["flb_stdout"]
    flb_stdout:
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/measurement/string_stamped"]

measurement_server:
  ros__parameters:
    measurement_plugins: ["my_string_stamped"]
    my_string_stamped:
      plugin: "dc_measurements/StringStamped"
      topic_output: "/dc/measurement/my_string_stamped"
      topic: "/hello-world"
      tags: ["flb_stdout"]
      enable_validator: false
      timer_based: false                                 # Get all data published on the input topic. Ignores polling_interval
```

Now that you know how it works, you can set your own measurements and destinations.
