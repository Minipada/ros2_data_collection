# Standalone CMake for fast local iteration on the ROS-independent core (no ROS/colcon
# needed) — mirrors what ament_add_gtest will build in the package CMakeLists. Not part
# of the ament build; used via `cmake -S dc_bridge/test/local` in the dev container.
cmake_minimum_required(VERSION 3.16)
project(dc_bridge_core_localtest CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tomlplusplus REQUIRED)
# msgpack-cxx is header-only; the Debian package ships msgpack-cxxConfig.cmake
find_package(msgpack-cxx QUIET)

set(CORE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_library(dc_bridge_core
  ${CORE_ROOT}/src/forwarder.cpp
  ${CORE_ROOT}/src/readiness.cpp
  ${CORE_ROOT}/src/topic_config.cpp
  ${CORE_ROOT}/src/supervisor.cpp
  ${CORE_ROOT}/src/vector_binary.cpp
  ${CORE_ROOT}/src/render.cpp
  ${CORE_ROOT}/src/uploader/content_type.cpp
  ${CORE_ROOT}/src/uploader/group.cpp
  ${CORE_ROOT}/src/uploader/status.cpp
  ${CORE_ROOT}/src/uploader/multipart.cpp
  ${CORE_ROOT}/src/uploader/intent_queue.cpp
  ${CORE_ROOT}/src/uploader/uploader.cpp
  ${CORE_ROOT}/src/uploader/retention.cpp)
target_include_directories(dc_bridge_core PUBLIC ${CORE_ROOT}/include)
target_link_libraries(dc_bridge_core PUBLIC nlohmann_json::nlohmann_json tomlplusplus::tomlplusplus)
if(TARGET msgpack-cxx)
  target_link_libraries(dc_bridge_core PUBLIC msgpack-cxx)
endif()

enable_testing()
foreach(t forwarder supervisor render misc uploader intent_queue retention)
  add_executable(${t}_test ${CORE_ROOT}/test/${t}_test.cpp)
  target_link_libraries(${t}_test dc_bridge_core GTest::gtest GTest::gtest_main Threads::Threads)
  target_compile_definitions(${t}_test PRIVATE
    DC_BRIDGE_FIXTURES_DIR="${CORE_ROOT}/test/fixtures/render")
  add_test(NAME ${t}_test COMMAND ${t}_test)
endforeach()
