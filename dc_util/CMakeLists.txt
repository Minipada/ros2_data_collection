# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

cmake_minimum_required(VERSION 3.16)
project(dc_util)

set(dependencies
  dc_common
  lifecycle_msgs
  nav2_ros_common
  nav2_util
  nlohmann_json
  rclcpp
  rclcpp_lifecycle
  yaml_cpp_vendor
)
# CONFIG, not the default module lookup: CMake 4 deprecates FindBoost (CMP0167) and Boost
# >= 1.70 ships its own config. Boost::headers is linked PUBLIC so consumers of the exported
# target get the include dirs (string_utils.hpp includes boost/ headers directly); ament can't
# carry the dep for them (it exports bare package names, which would re-run the deprecated
# module), so dc_util-extras.cmake finds Boost CONFIG-first in every consumer.
find_package(Boost CONFIG REQUIRED)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

# ament_target_dependencies was removed in lyrical (#531): dependencies link by exported
# target instead. Message packages have no pkg::pkg umbrella target on lyrical -- their
# C++ typesupport target carries the generated code; dc_common is header-only;
# yaml_cpp_vendor exports no target (yaml-cpp headers come via ${YAML_CPP_INCLUDEDIR}).
dc_package()

set(library_name base64_library)

add_library(${library_name} SHARED
  src/base64.cpp
)

target_include_directories(${library_name} PUBLIC ${dc_common_INCLUDE_DIRS})
target_link_libraries(${library_name} PUBLIC
  Boost::headers
  lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_cpp
  nav2_ros_common::nav2_ros_common
  nav2_util::nav2_util_core
  nlohmann_json::nlohmann_json
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Namespaced export so consumers link dc_util::base64_library on every distro.
set(export_name export_${library_name})

ament_export_targets(${export_name} HAS_LIBRARY_TARGET NAMESPACE dc_util::)

install(TARGETS ${library_name}
  EXPORT ${export_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package(CONFIG_EXTRAS "dc_util-extras.cmake")
