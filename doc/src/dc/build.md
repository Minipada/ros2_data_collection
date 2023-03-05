# Build

## Download the repository
Given that there is no apt packages available, you will need to build from source.

Download the repository in your workspace

```bash
$ git clone github.com/minipada/ros2_data_collection.git
```

## Install dependencies
### Install system and C/C++ dependencies
```
$ rosdep install --from-paths src --ignore-src -r -y
```

Some packages are external C++ packages but a vendor package has been included in the repository so colcon will handle it.
In addition, since this set of packages has no python external dependencies, you won't need anything else.

### Install python dependencies
This project uses [poetry](https://python-poetry.org/) to manage python dependencies. It is easy to use and it is possible to set each package version like in a requirements.txt and manage multiple python environments.
If you have poetry on your machine, you can execute:

`$ poetry install`

If not, you can install python dependencies from the provided requirements.txt:

`$ pip3 install -r requirements.txt`

## Build

```bash
$ colcon build
```

## Run
```bash
source install/setup.bash
ros2 launch dc_bringup bringup.launch.py
```

## Issues
If you run into any issues when building ROS 2 Data Collection, you can use the search tool in the issues tab on [GitHub](https://github.com/minipada/ros2_data_collection) and always feel free to [open a ticket](https://github.com/minipada/ros2_data_collection).
