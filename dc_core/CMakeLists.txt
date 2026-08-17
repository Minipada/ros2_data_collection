# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

cmake_minimum_required(VERSION 3.8)
project(dc_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  dc_common
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

dc_package()

install(
  DIRECTORY include/
  DESTINATION include/
)

# condition_set.hpp is deliberately ROS-free, so its test needs no node, no rclcpp::init and none
# of the dependencies above -- just the headers.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_condition_set test/test_condition_set.cpp)
  target_include_directories(${PROJECT_NAME}_test_condition_set PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
