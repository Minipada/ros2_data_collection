# ghcr.io/minipada/ros2_data_collection:${ROS_DISTRO}-ci-testing
# CI image using the ROS testing repository

ARG ROS_DISTRO=humble
FROM ghcr.io/minipada/ros2_data_collection:${ROS_DISTRO}-ci

# Switch to ros-testing
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN echo "deb http://packages.ros.org/ros2-testing/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2-latest.list

# Upgrade packages to ros-shadow-fixed and clean apt-cache within one RUN command
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
