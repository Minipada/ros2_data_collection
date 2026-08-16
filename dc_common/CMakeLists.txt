cmake_minimum_required(VERSION 3.5)
project(dc_common)

find_package(ament_cmake REQUIRED)

# dc_common defines dc_package(); it can't find_package() itself the way every other dc_*
# package does, so it pulls the macro in directly here to apply the same standard project
# setup (C++17, warnings, ...) to its own build.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/dc_package.cmake)
dc_package()

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

# RecordRingBuffer / FileScratchRing (#285): header-only, ROS-free utilities, same shape as
# dc_core's condition_set.hpp -- no compiled library needed, just the headers on the include
# path.
install(
  DIRECTORY include/
  DESTINATION include/
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  foreach(test record_ring_buffer_test file_scratch_ring_test)
    ament_add_gtest(${PROJECT_NAME}_${test} test/${test}.cpp)
    target_include_directories(${PROJECT_NAME}_${test} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
  endforeach()
endif()

ament_package(
  CONFIG_EXTRAS "dc_common-extras.cmake"
)
