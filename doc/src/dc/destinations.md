# Overview

## Description
A destination is where the data will be sent: AWS S3, stdout, AWS Kinesis. It has the possibility to use [outputs from fluentbit](https://docs.fluentbit.io/manual/pipeline/outputs).

The destination node is similar to the measurement one. It:
* Either subscribes to data in the main node and data is forwarded with ros based destination plugins
* Either subscribes to data from the ros2 fluent bit plugin.

The fluent bit plugin is the preferred one since when using it, we get all the benefits from fluent bit (especially data integrity). The ros2 fluent bit plugin uses rclc and is at the moment a fork of fluent bit. It could also be rewritten as a fluent bit plugin based on the GO interface, but pros and cons to move to it are not clear yet.


## Node parameters

Destinations parameters are loaded dynamically. Here are the static ones:

| Parameter name                                                                                                                                | Description                                                                                                                                                                                                                                           | Type(s)     | Default                 |
| --------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------- | ----------------------- |
| [flb.flush](https://docs.fluentbit.io/manual/v/1.3/service)                                                                                   | Interval to flush output (seconds)                                                                                                                                                                                                                    | str         | "1"                     |
| [flb.grace](https://docs.fluentbit.io/manual/v/1.3/service)                                                                                   | Wait time (seconds) on exit                                                                                                                                                                                                                           | str         | "1"                     |
| [flb.log_level](https://docs.fluentbit.io/manual/v/1.3/service)                                                                               | Diagnostic level (error/warning/info/debug/trace)                                                                                                                                                                                                     | str         | "info"                  |
| [flb.storage_path](https://docs.fluentbit.io/manual/administration/buffering-and-storage)                                                     | Set an optional location in the file system to store streams and chunks of data. If this parameter is not set, Input plugins can only use in-memory buffering.                                                                                        | str         | "/var/log/flb-storage/" |
| [flb.storage_sync](https://docs.fluentbit.io/manual/administration/buffering-and-storage)                                                     | Configure the synchronization mode used to store the data into the file system. It can take the values normal or full.                                                                                                                                | str         | "normal"                |
| [flb.storage_checksum](https://docs.fluentbit.io/manual/administration/buffering-and-storage)                                                 | Enable the data integrity check when writing and reading data from the filesystem. The storage layer uses the CRC32 algorithm.                                                                                                                        | str         | "off"                   |
| [flb.storage_backlog_mem_limit](https://docs.fluentbit.io/manual/administration/buffering-and-storage)                                        | If storage.path is set, Fluent Bit will look for data chunks that were not delivered and are still in the storage layer, these are called backlog data. This option configure a hint of maximum value of memory to use when processing these records. | str         | "5M"                    |
| [flb.scheduler_cap](https://docs.fluentbit.io/manual/administration/scheduling-and-retries)                                                   | Set a maximum retry time in seconds. The property is supported from v1.8.7.                                                                                                                                                                           | str         | "2000"                  |
| [flb.scheduler_base](https://docs.fluentbit.io/manual/administration/scheduling-and-retries)                                                  | Set a base of exponential backoff. The property is supported from v1.8.7.                                                                                                                                                                             | str         | "5"                     |
| [flb.http_server](https://docs.fluentbit.io/manual/administration/buffering-and-storage)                                                      | If true enable statistics HTTP server                                                                                                                                                                                                                 | bool        | false                   |
| [flb.in_storage_type](https://docs.fluentbit.io/manual/administration/buffering-and-storage#input-section-configuration)                      | Specifies the buffering mechanism to use. It can be memory or filesystem.                                                                                                                                                                             | str         | "filesystem"            |
| [flb.in_storage_pause_on_chunks_overlimit](https://docs.fluentbit.io/manual/administration/buffering-and-storage#input-section-configuration) | Specifies if file storage is to be paused when reaching the chunk limit.                                                                                                                                                                              | str         | "off"                   |
| custom_str_params_list                                                                                                                        | Custom strings to use in other parameters. They are also appended in the json sent to the destination                                                                                                                                                 | list\[str\] | N/A                     |
| run_id.enabled                                                                                                                                | Identify which run the robot is. A new one is generated at every start of the node. Uses either a counter or UUID                                                                                                                                     | str         | true                    |
| run_id.counter                                                                                                                                | Enable counter for the run_id                                                                                                                                                                                                                         | str         | true                    |
| run_id.counter_path                                                                                                                           | Path to store the last run. It is expanded with environment variables id                                                                                                                                                                              | str         | "$HOME/run_id"          |
| run_id.uuid                                                                                                                                   | Generate a new run ID by using a random UUID                                                                                                                                                                                                          | str         | false                   |


## Plugin parameters

| Parameter name | Description                                       | Type(s)                  | Default  |
| -------------- | ------------------------------------------------- | ------------------------ | -------- |
| plugin         | Plugin to load                                    | list\[str\]              | N/A      |
| inputs         | Topics to which to listen to get the data         | list\[str\]              | N/A      |
| debug          | Enable debug print                                | bool                     | false    |
| time_format    | Format the data will be printed                   | str("double", "iso8601") | "double" |
| time_key       | Dictionary key from which date will be taken from | str                      | "date"   |

## Available plugins:

| Name                                                              | Description                                |
| ----------------------------------------------------------------- | ------------------------------------------ |
| [flb_file](./destinations/flb_file.md)                            | JSON in a file via Fluent bit              |
| [flb_http](./destinations/flb_http.md)                            | JSON to http request via Fluent bit        |
| [flb_kinesis_streams](./destinations/flb_kinesis_data_streams.md) | JSON to AWS Kinesis Streams via Fluent bit |
| [flb_minio](./destinations/flb_minio.md)                          | JSON to Minio via Fluent bit               |
| [flb_pgsql](./destinations/flb_pgsql.md)                          | JSON to PostgreSQL via Fluent bit          |
| [flb_stdout](./destinations/flb_stdout.md)                        | JSON to STDOUT via Fluent bit              |
| [rcl](./destinations/rcl.md)                                      | JSON to RCL                                |


## Example configuration

```yaml
destination_server:
  ros__parameters:
    flb:
      flush: "1"
      flb_grace: "1"
      log_level: "info"
      storage_path: "/var/log/flb-storage/"
      storage_sync: "full"
      storage_checksum: "off"
      storage_backlog_mem_limit: "1M"
      scheduler_cap: "2000"
      scheduler_base: "5"
      metrics: true
      in_storage_type: "filesystem"
      in_storage_pause_on_chunks_overlimit: "off"
    destination_plugins: ["flb_stdout"]
    custom_str_params_list: ["robot_name", "id"]
    custom_str_params:
      robot_name:
        name: robot_name
        value: david-pc
      id:
        name: id
        value_from_file: /etc/machine-id
    run_id:
      enabled: true
      counter: true
      counter_path: "$HOME/run_id"
      uuid: false
    flb_minio:
      plugin: "dc_destinations/FlbMinIO"
      inputs: ["/dc/group/map"]
      plugin_path: "/root/ws/src/ros2_data_collection/dc_destinations/flb_plugins/lib/out_minio.so"
      endpoint: 127.0.0.1:9000
      access_key_id: HgJdDWeDQBiBWCwm
      secret_access_key: plCMROO2VMZIKiqEwDd80dLJUCRvJ9iu
      use_ssl: false
      bucket: "mybucket"
      src_fields: ["camera.local_img_paths.raw","camera.local_img_paths.rotated", "map.local_map_paths.yaml", "map.local_map_paths.pgm"]
      upload_fields: ["camera.minio_img_paths.raw","camera.minio_img_paths.rotated", "map.minio_map_paths.yaml", "map.minio_map_paths.pgm"]
    flb_pgsql:
      plugin: "dc_destinations/FlbPgSQL"
      inputs: ["/dc/group/memory_uptime"]
      host: "127.0.0.1"
      port: "5432"
      user: fluentbit
      password: password
      database: "fluentbit"
      table: "dc"
      timestamp_key: "date"
      async: false
      time_format: "double"
      time_key: "date"
    rcl:
      plugin: "dc_destinations/Rcl"
      inputs: ["/dc/group/memory_uptime"]
    flb_stdout:
      plugin: "dc_destinations/FlbStdout"
      inputs: ["/dc/group/cameras"]
      inputs: ["/dc/group/cameras"]
      time_format: "iso8601"
      time_key: "date"
    flb_http:
      plugin: "dc_destinations/FlbHTTP"
      inputs: ["/dc/group/memory_uptime"]
      debug: true
    flb_file:
      plugin: "dc_destinations/FlbFile"
      inputs: ["/dc/group/memory_uptime"]
      path: "$HOME/data"
      file: uptime
      debug: false
      throttle:
        enable: false
        rate: "1"
        window: "5"
        interval: "10s"
        print_status: true
```
