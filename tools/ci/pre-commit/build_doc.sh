#!/bin/bash

set -e

# Build the documentation

# Arguments parsing
# Example:
# ./build_doc.sh -ros-version="humble"
help() {
usage="$(basename "$0") --ros-version=<ROS_VERSION> [ --help ]

Build documentation

Arguments:
--help          Show this help text
--ros-version   ROS version to use, will use different containers (mandatory)
"
    echo "${usage}"
}

for i in "$@"
do
case $i in
    --ros-version=*)
    ROS_VERSION="${i#*=}"
    shift # past argument=value
    ;;
    --help)
    HELP=YES
    shift # past argument with no value
    ;;
    *)
    # unknown option
    ;;
esac
done

# Arguments check
if [ -n "${HELP}" ]; then
    help
    exit 0
fi

if [ -z "${ROS_VERSION}" ]; then
    help
    exit 1
fi

docker run --rm -v "${PWD}:/ws" --workdir /ws/doc "minipada/ros2_data_collection:${ROS_VERSION}-doc" mdbook build
