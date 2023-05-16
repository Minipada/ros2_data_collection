# DC

<p class="github-only" align="center">
  <img height="300" src="./doc/src/images/dc.png" />
</p>

**Documentation**: [https://minipada.github.io/ros2_data_collection](https://minipada.github.io/ros2_data_collection)

**Source code**: [https://github.com/minipada/ros2_data_collection](https://github.com/minipada/ros2_data_collection)

[![ROS 2](https://img.shields.io/badge/ROS%202-humble-informational?style=for-the-badge)](https://docs.ros.org/en/humble/index.html) ![python](https://img.shields.io/badge/python-3.10.4-informational?style=for-the-badge) ![C++](https://img.shields.io/badge/C++-17-informational?style=for-the-badge)

[![codecov](https://codecov.io/gh/Minipada/ros2_data_collection/branch/humble/graph/badge.svg?token=Y2UA5OE0KR)](https://codecov.io/gh/Minipada/ros2_data_collection) [![tests](https://minipada.testspace.com/spaces/219054/badge?token=8214fc76eff8c09b47136742d644d2a1ac0e38e3)](https://minipada.testspace.com/spaces/219054?utm_campaign=badge&utm_medium=referral&utm_source=test)

| Humble                                                                                                                                                                                                                        |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [![Docker](https://github.com/minipada/ros2_data_collection/actions/workflows/docker.yaml/badge.svg)](https://github.com/minipada/ros2_data_collection/actions/workflows/docker.yaml)                                         |
| [![Format](https://github.com/minipada/ros2_data_collection/actions/workflows/format.yaml/badge.svg)](https://github.com/minipada/ros2_data_collection/actions/workflows/format.yaml)                                         |
| [![Documentation](https://github.com/minipada/ros2_data_collection/actions/workflows/doc.yaml/badge.svg)](https://github.com/minipada/ros2_data_collection/actions/workflows/doc.yaml)                                        |
| [![Github Pages](https://github.com/Minipada/ros2_data_collection/actions/workflows/pages/pages-build-deployment/badge.svg)](https://github.com/Minipada/ros2_data_collection/actions/workflows/pages/pages-build-deployment) |
| [![CI](https://github.com/Minipada/ros2_data_collection/actions/workflows/ci.yaml/badge.svg)](https://github.com/Minipada/ros2_data_collection/actions/workflows/ci.yaml)                                                     |


For detailed instructions:

- [Build](https://minipada.github.io/ros2_data_collection/dc/setup.html)
- [Concepts](https://minipada.github.io/ros2_data_collection/dc/concepts.html)
- [Data Pipeline](https://minipada.github.io/ros2_data_collection/dc/data_pipeline.html)
- [Measurements](https://minipada.github.io/ros2_data_collection/dc/measurements.html)
- [Conditions](https://minipada.github.io/ros2_data_collection/dc/conditions.html)
- [Data validation](https://minipada.github.io/ros2_data_collection/dc/data_validation.html)
- [Groups](https://minipada.github.io/ros2_data_collection/dc/groups.html)
- [Destinations](https://minipada.github.io/ros2_data_collection/dc/destinations.html)
- [Configuration examples](https://minipada.github.io/ros2_data_collection/dc/configuration_examples.html)
- [Getting started](https://minipada.github.io/ros2_data_collection/dc/getting_started.html)
- [Infrastructure setup](https://minipada.github.io/ros2_data_collection/dc/infrastructure_setup.html)
- [CLI tools](https://minipada.github.io/ros2_data_collection/dc/cli.html)
- [Future work and Roadmap](https://minipada.github.io/ros2_data_collection/dc/future_work.html)
- [Contributing](https://minipada.github.io/ros2_data_collection/dc/contributing.html)
- [FAQ](https://minipada.github.io/ros2_data_collection/dc/faq.html)
- [About and contact](https://minipada.github.io/ros2_data_collection/dc/about_contact.html)

## Introduction

The DC (Data Collection) project aims at integrating data collection pipelines into ROS 2. The goal is to integrate data collection pipelines with existing APIs to enable data analytics, rather than live monitoring, which already has excellent tools available. As companies increasingly turn to autonomous robots, the ability to understand and improve operations for any type of machine in any environment has become crucial. This involves mostly pick and drop and inspection operations. This framework aims at helping collecting, validating (through JSON schemas) and sending reliably the data to create such APIs and dashboards.

DC uses a modular approach, based on [pluginlib](https://index.ros.org/p/pluginlib/) and greatly inspired by [Nav2](https://navigation.ros.org/) for its architecture. Pluginlib is used to configure which measurements are collected and where the data goes. Measurements and destinations are pluginlib plugins. In addition to pluginlib, most plugins use [Fluent bit](https://fluentbit.io/) in the backend: *Fluent Bit is a super fast, lightweight, and highly scalable logging and metrics processor and forwarder. It is the preferred choice for cloud and containerized environments. Developed and interfaced in C, it has already many features we directly can use, especially: high performance, reliability and data integrity (backpressure handling and data buffering in memory and filesystem)*.

## Why collect data from robots?

1. **Performance Monitoring**: Collecting data from a robot allows you to monitor its performance and identify areas for improvement. For example, you can use data to analyze the robot's motion and identify areas where it may be experiencing issues or inefficiencies.
2. **Fault Diagnosis**: Data collection can also be used to diagnose faults and troubleshoot issues with the robot. By collecting data on various aspects of the robot's behavior, you can identify patterns or anomalies that may indicate problems with the system.
3. **Machine Learning**: Data collected from robots can be used to train machine learning models, which can be used to improve the robot's performance and behavior. For example, you can use data collected from sensors to train models for object detection or path planning.
4. **Research and Development**: Data collection is important for research and development in robotics. By collecting data on the behavior of robots in different scenarios, researchers can gain insights into how robots can be designed and optimized for different applications.
5. **Inventory Management**: Data collection can be used to monitor inventory levels and track the movement of goods within a warehouse. This can help managers identify which products are in high demand and optimize the placement of products to improve order fulfillment times.
6. **Resource Allocation**: Data collection can also help managers allocate resources more efficiently. For example, by monitoring the movement of people and goods within a warehouse, managers can identify bottlenecks and areas of congestion and adjust staffing and equipment allocation to address these issues.
7. **Process Improvement**: Data collection can be used to monitor and analyze the performance of various processes within a warehouse. By identifying areas of inefficiency or errors, managers can develop strategies for improving these processes and increasing productivity.
8. **Predictive Maintenance**: Data collection can be used to monitor the performance of equipment and identify potential maintenance issues before they occur. This can help managers schedule maintenance more effectively and avoid costly downtime due to equipment failure.

## Main features

* **Open source**: Currently all tools on the market are not open source. This project is in [MPL-2.0 license](https://www.mozilla.org/en-US/MPL/2.0/), in summary you can use without asking permission and without paying
* **Modular approach**: based on pluginlib and greatly inspired by Nav2 for its architecture
* **Reliable data collection**: validate and send data to create APIs and dashboards
* **Flexible data collection**: set polling interval for each measurement or collect every measurement with StringStamped messages
* **Customizable validation**: validate data using existing or customized JSON schemas
* **Easy to extend**: add new measurements or destinations by simply adding a plugin
* **Flexible data collection conditions**: collect data based on conditions such as whether the robot is moving or if a field is equal to a value
* **Trigger-based data collection**: collect data when a defined set of combination of all, any, or no condition are met
* **Customizable record collection**: configure the number of records to collect at the start and when a condition is activated.
* **Data inspection**: inspect data from camera input including barcode and QR codes
* **Fast and efficient**: high performance, using Fluent Bit for backend processing, and designed to minimize code duplication and reduce human errors
* **Grouped measurements**: measurements can be grouped using the group node based on the ApproximateTimeSynchronizer
* **File saving**: files can be saved, including map_server maps, camera images, and any file produced by a measurement
* **Easy to use**: designed to be easy to learn and use
* **No C++ 3rd party library required**: all 3rd party libraries have a vendor package in the repository

And inherited from Fluent Bit:

* [Backpressure handling](https://docs.fluentbit.io/manual/v/1.0/configuration/backpressure)
* [Data buffering in memory and filesystem](https://docs.fluentbit.io/manual/v/1.0/configuration/buffering)

Here is an example of a pipeline:

```mermaid
flowchart LR
    pl_camera1["Camera bottom"]
    pl_camera2["Camera middle"]
    pl_camera3["Camera top"]
    pl_condition_moving["Moving"]
    pl_cpu["CPU"]
    pl_cmd_vel["Command velocity"]
    pl_memory["Memory"]
    pl_position["Position"]
    pl_speed["Speed"]
    pl_storage["Storage"]
    pl_uptime["Uptime"]
    pl_network["Network"]
    pl_network_boot["Network"]
    pl_os_boot["OS"]

    subgraph m_n["Measurement node"]
        subgraph cond["Condition plugins"]
            pl_condition_moving
        end
        subgraph measurements["Measurement plugins"]
            pl_camera1
            pl_camera2
            pl_camera3
            pl_cpu
            pl_cmd_vel
            pl_memory
            pl_position
            pl_speed
            pl_storage
            pl_uptime
            pl_network
            pl_network_boot
            pl_os_boot
        end
    end

    subgraph g_n["Group node"]
        gr_boot_system["System (boot)"]
        gr_system["System"]
        gr_robot["Robot"]
        gr_inspection["Inspection"]
    end

    subgraph d_n["Destination node"]
        pl_pgsql["PostgreSQL"]
        pl_minio["Minio"]
        pl_s3["S3"]
    end

    pl_camera1 -- if not --> pl_condition_moving --> gr_inspection
    pl_camera2 -- if not --> pl_condition_moving --> gr_inspection
    pl_camera3 -- if not --> pl_condition_moving --> gr_inspection
    pl_cpu --> gr_system
    pl_memory --> gr_system
    pl_uptime --> gr_boot_system
    pl_network_boot --> gr_boot_system
    pl_os_boot --> gr_boot_system
    pl_storage --> gr_system
    pl_cmd_vel --> gr_robot
    pl_position --> gr_robot
    pl_speed --> gr_robot

    pl_network -- Network ping and online status --> pl_pgsql
    gr_boot_system -- os, network interfaces\n, permissions and uptime --> pl_pgsql
    gr_robot -- Robot cmd_vel, position. speed --> pl_pgsql
    gr_system -- Available space,\n memory used and cpu usage --> pl_pgsql
    gr_inspection -- Image paths on s3 and minio --> pl_pgsql
    gr_inspection -- Raw, rotated and/or inspected images --> pl_minio
    gr_inspection -- Raw, rotated and/or inspected images --> pl_s3
```

# License
This program is under the terms of the [Mozilla Public License Version 2.0](https://www.mozilla.org/en-US/MPL/2.0/).

# About and Contact

For any inquiry, please contact David ([d.bensoussan@proton.me](mailto:d.bensoussan@proton.me)). If your inquiry relates to bugs or open-source feature requests, consider posting a ticket on our GitHub project. If your inquiry relates to configuration support or private feature development, reach out and we will be able to support you in your projects.
