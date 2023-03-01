# Data Pipeline

```mermaid
flowchart LR
    dc_bringup_launch["DC Bringup Launch"]
    yaml_param["YAML Parameter file"]
    ros_middleware["ROS middleware"]

    subgraph m_s["Measurement Server"]
        direction LR
        meas_node["Measurement Node"]
        measurement_plugins["Measurement plugins"]
        meas_topics_out["ROS Topics"]
    end

    subgraph d_s["Destination Server"]
        direction LR
        dest_node["Destination Node"]
    end

    subgraph g_s["Group Server"]
        direction LR
        group_node["Group Node"]
        group_grouped_data["Grouped data"]
        group_topics_out["ROS Topics"]
    end

    subgraph flb["Fluent Bit"]
        direction LR
        flb_server["Fluent Bit Server"]
        flb_mem_buff["Memory buffer"]
        flb_storage_buff["Storage buffer"]
        flb_record["Record"]
        flb_ros2_plugin["ROS2 plugin"]
        flb_output["Outputs"]
    end

    dc_bringup_launch--starts-->meas_node
    dc_bringup_launch--starts-->dest_node
    dc_bringup_launch--starts-->group_node
    dc_bringup_launch--loads-->yaml_param

    meas_node--initializes-->measurement_plugins
    meas_node--"publishes to"-->meas_topics_out

    dest_node--starts-->flb_server
    dest_node--loads-->flb_ros2_plugin

    flb_ros2_plugin--"subscribes to"-->meas_topics_out
    flb_ros2_plugin--"subscribes to"-->group_topics_out
    group_node--"publishes"-->meas_topics_out
    group_node--generates-->group_grouped_data
    group_grouped_data-->group_topics_out

    flb_ros2_plugin--generates-->flb_record
    flb_record--"stores in"-->flb_mem_buff
    flb_record--"stores in"-->flb_storage_buff
    flb_mem_buff--flushes-->flb_output
    flb_storage_buff--flushes-->flb_output

    group_topics_out--"sends to"-->ros_middleware
    meas_topics_out--"sends to"-->ros_middleware
    ros_middleware--"sends to"-->flb_ros2_plugin
```

The data flow precised in this flowchart summarizes how data moves:
1. DC Bringup loads the yaml configuration file
2. DC Bringup starts the measurement node
   1. Measurement plugins are loaded and data starts to be collected
   2. Each plugin publishes what it collected on a ROS topic
3. DC Bringup starts the destination node
   1. Destination plugins are loaded
   2. Fluent Bit server is started: as soon as it receives records, it goes through filters and then is flushed to desired destinations
   3. ROS2 Fluent Bit plugin is loaded: it subscribes to measurement and group output topics
   4. Fluent Bit filters are initialized: it edits data received, modify the tags to match the desired output(s) and edit the timestamp field if required
4. DC Bringup starts the group node
   1. Subscribes to measurement and group topics outputs
   2. When all measurements of a group are received, it publishes the grouped data on another group
