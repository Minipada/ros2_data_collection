# ghcr.io/minipada/dc:${ROS_DISTRO}-ci
# ROS base image augmented with all DC dependencies to use for CI

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Setup (temporary) ROS workspace
WORKDIR /root/dc

# Copy DC sources from docker context
COPY . src/dc

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
# hadolint ignore=DL3003
RUN \
    # Update apt package list as previous containers clear the cache
    apt-get -q update && \
    apt-get -q -y dist-upgrade && \
    #
    # Install some base dependencies
    apt-get -q install --no-install-recommends -y \
    # Some basic requirements
    wget git sudo curl \
    # Python3
    python3-pip \
    # Fluent bit dependencies
    libcxsparse3 \
    # Fluent bit postgresql
    postgresql-server-dev-all \
    # Preferred build tools
    clang clang-format-12 clang-tidy clang-tools graphviz-dev libxml2-utils libzbar0 \
    ccache && \
    # Install Python dependencies
    pip install --no-cache-dir testresources poetry==1.1.13 && \
    cd src/dc && \
    poetry config virtualenvs.create false --local && \
    poetry install && \
    apt-get remove --purge python3-pip -y && \
    cd ../.. && \
    # Fetch all dependencies from dc.repos
    if [ -r "src/dc/dc_${ROS_DISTRO}.repos" ] ; then vcs import src < src/dc/dc.repos ; fi && \
    if [ -r "src/dc/dc_${ROS_DISTRO}.repos" ] ; then vcs import src < "src/dc/dc_${ROS_DISTRO}.repos" ; fi && \
    # Download all dependencies of DC
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install -y --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" --as-root=apt:false && \
    rm -rf src && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
