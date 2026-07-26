cmake_minimum_required(VERSION 3.16)
project(dc_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dc_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tomlplusplus REQUIRED)
find_package(msgpack-cxx REQUIRED)
find_package(Threads REQUIRED)

# ROS-independent core (Forwarder/Supervisor/Readiness/TopicConfig/ConfigRenderer/
# vector_binary). No ROS dependency, so it's unit-tested with plain gtest below.
add_library(dc_bridge_core
  src/forwarder.cpp
  src/readiness.cpp
  src/topic_config.cpp
  src/supervisor.cpp
  src/vector_binary.cpp
  src/render.cpp)
target_include_directories(dc_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dc_bridge_core PUBLIC
  nlohmann_json::nlohmann_json
  tomlplusplus::tomlplusplus
  msgpack-cxx
  Threads::Threads)

# The ROS node.
add_executable(dc_bridge src/main.cpp src/bridge_node.cpp)
target_link_libraries(dc_bridge dc_bridge_core)
ament_target_dependencies(dc_bridge rclcpp dc_interfaces std_msgs std_srvs)

install(TARGETS dc_bridge DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t forwarder supervisor render misc)
    ament_add_gtest(${t}_test test/${t}_test.cpp)
    target_link_libraries(${t}_test dc_bridge_core)
    target_compile_definitions(${t}_test PRIVATE
      DC_BRIDGE_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/render")
  endforeach()
endif()

ament_package()
