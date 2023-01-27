# syntax = docker/dockerfile:1.3

# TODO Improve following https://github.com/ros-planning/navigation2/blob/main/tools/source.Dockerfile

# ghcr.io/minipada/dc:${ROS_DISTRO}-source
# Downloads the DC source code and install remaining debian dependencies

ARG ROS_DISTRO=humble
FROM ghcr.io/minipada/dc:${ROS_DISTRO}-source

# Export ROS_UNDERLAY for downstream docker containers
ENV ROS_UNDERLAY /root/dc/install
WORKDIR $ROS_UNDERLAY/..

# Copy MoveIt sources from docker context
COPY . src/dc

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN --mount=type=cache,target=/root/.ccache/ \
    # Enable ccache
    PATH=/usr/lib/ccache:$PATH && \
    # Fetch required upstream sources for building
    if [ -r "src/dc/dc_${ROS_DISTRO}.repos" ] ; then vcs import src < src/dc/dc_sim.repos ; fi  && \
    if [ -r "src/dc/dc_${ROS_DISTRO}.repos" ] ; then vcs import src < "src/dc/dc_sim_${ROS_DISTRO}.repos" ; fi && \
    #
    apt-get update && \
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install -y --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" --as-root=apt:false && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build \
    # --packages-up-to-regex 'dc.*' \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers desktop_notification- status- && \
    ccache -s && \
    rm -rf /var/lib/apt/lists/* && \
    #
    # Update /ros_entrypoint.sh to source our new workspace
    sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/setup.sh#g" /ros_entrypoint.sh