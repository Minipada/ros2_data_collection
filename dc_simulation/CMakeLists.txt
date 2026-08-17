# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

cmake_minimum_required(VERSION 3.5)
project(dc_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY config launch maps models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/dc_simulation.dsv.in")

ament_export_dependencies(ament_cmake)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
