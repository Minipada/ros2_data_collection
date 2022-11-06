cmake_minimum_required(VERSION 3.5)
project(dc_common)

find_package(ament_cmake_core REQUIRED)

ament_package(
  CONFIG_EXTRAS "dc_common-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
