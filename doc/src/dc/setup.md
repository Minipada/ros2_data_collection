# Setup

## Use docker

### Available images

Docker images with latest code are available on the [Docker public registry](https://hub.docker.com/repository/docker/minipada/ros2_data_collection):

| Image                                           | Description                                                     |
| ----------------------------------------------- | --------------------------------------------------------------- |
| minipada/ros2_data_collection:humble-ci         | ROS base image augmented with all DC dependencies to use for CI |
| minipada/ros2_data_collection:humble-ci-testing | CI image using the ROS testing repository                       |
| minipada/ros2_data_collection:humble-source     | DC source compiled                                              |
| minipada/ros2_data_collection:humble-source-sim | DC source compiled with all simulation packages                 |
| minipada/ros2_data_collection:humble-doc        | Documentation                                                   |

Get any by running:

```bash
$ docker pull minipada/ros2_data_collection:<TAG>
```

### Docker workflow

When developing, use the *source-sim* image with a docker compose file. The latest one is available on the repository:

```yaml
version: "3.9"

services:
  ros2-data-collection:
    image: minipada/ros2_data_collection:humble-source-sim
    privileged: true
    container_name: ros2-data-collection
    environment:
      - QT_X11_NO_MITSHM=1
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=compute,utility,display
      - DISPLAY=unix$DISPLAY
      - XAUTHORITY=$XAUTHORITY
    volumes:
      - ${HOME}/.Xauthority:/root/.Xauthority
      - /tmp/.X11-unix:/tmp/.X11-unix
      - /var/run/docker.sock:/var/run/docker.sock
      - ${PWD}:/root/ws/src/ros2_data_collection
    network_mode: "host"
    restart: "unless-stopped"
    stop_grace_period: "3s"
    runtime: nvidia
    command: tail -F anything
    working_dir: /root/ws
```

Then, to allow GUI (RViz, Gazebo) to work in the container, execute:

```bash
$ xhost +
```

Then start the container:

```bash
$ docker compose up -d
```

And get in the container with:
```bash
$ docker exec -it ros2-data-collection /ros_entrypoint.sh bash
```

## Build from source

### Download the repository
Given that there is no apt packages available, you will need to build from source.

Download the repository in your workspace

```bash
$ git clone github.com/minipada/ros2_data_collection.git
```

### Install dependencies
#### Install system and C/C++ dependencies
```
$ rosdep install --from-paths src --ignore-src -r -y
```

Some packages are external C++ packages but a vendor package has been included in the repository so colcon will handle it.
In addition, since this set of packages has no python external dependencies, you won't need anything else.

#### Install python dependencies
This project uses [poetry](https://python-poetry.org/) to manage python dependencies. It is easy to use and it is possible to set each package version like in a requirements.txt and manage multiple python environments.
If you have poetry on your machine, you can execute:

`$ poetry install`

If not, you can install python dependencies from the provided requirements.txt:

`$ pip3 install -r requirements.txt`

### Build

```bash
$ colcon build
```

### Run
```bash
source install/setup.bash
ros2 launch dc_bringup bringup.launch.py
```

## Issues
If you run into any issues when building ROS 2 Data Collection, you can use the search tool in the issues tab on [GitHub](https://github.com/minipada/ros2_data_collection) and always feel free to [open a ticket](https://github.com/minipada/ros2_data_collection).
