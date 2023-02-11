# Build

## Download the repository
Given that there is no apt packages available, you will need to build from source.

Download the repository in your workspace

```bash
$ git clone github.com/brisa-robotics/ros2_data_collection.git
```

## Install dependencies

```
$ rosdep install --from-paths src --ignore-src -r -y
```

Some packages are external C++ packages but a vendor package has been included in the repository so colcon will handle it.
In addition, since this set of packages has no python external dependencies, you won't need anything else.

## Build

```bash
$ colcon build
```

## Issues
If you run into any issues when building ROS2 Data Collection, you can use the search tool in the issues tab on [GitHub](https://github.com/minipada/ros2_data_collection) and always feel free to [open a ticket](https://github.com/minipada/ros2_data_collection).
