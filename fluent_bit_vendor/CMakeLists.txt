cmake_minimum_required(VERSION 3.5)

project(fluent_bit_vendor)

find_package(ament_cmake REQUIRED)
find_package(dc_interfaces REQUIRED)

set(cmake_commands)
set(cmake_configure_args -Wno-dev)

if(WIN32)
  if(DEFINED CMAKE_GENERATOR)
    list(APPEND cmake_configure_args -G ${CMAKE_GENERATOR})
  endif()
  if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^(x86_|x86-|AMD|amd|x)64$")
    list(APPEND cmake_configure_args -A x64)
  endif()
endif()

if(DEFINED CMAKE_BUILD_TYPE)
  if(WIN32)
    build_command(_build_command CONFIGURATION ${CMAKE_BUILD_TYPE})
    list(APPEND cmake_commands "BUILD_COMMAND ${_build_command}")
  else()
    list(APPEND cmake_configure_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()
endif()

if(DEFINED CMAKE_TOOLCHAIN_FILE)
  list(APPEND cmake_configure_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  if(ANDROID)
    if(DEFINED ANDROID_ABI)
      list(APPEND cmake_configure_args "-DANDROID_ABI=${ANDROID_ABI}")
    endif()
    if(DEFINED ANDROID_CPP_FEATURES)
      list(APPEND cmake_configure_args "-DANDROID_CPP_FEATURES=${ANDROID_CPP_FEATURES}")
    endif()
    if(DEFINED ANDROID_FUNCTION_LEVEL_LINKING)
      list(APPEND cmake_configure_args "-DANDROID_FUNCTION_LEVEL_LINKING=${ANDROID_FUNCTION_LEVEL_LINKING}")
    endif()
    if(DEFINED ANDROID_NATIVE_API_LEVEL)
      list(APPEND cmake_configure_args "-DANDROID_NATIVE_API_LEVEL=${ANDROID_NATIVE_API_LEVEL}")
    endif()
    if(DEFINED ANDROID_NDK)
      list(APPEND cmake_configure_args "-DANDROID_NDK=${ANDROID_NDK}")
    endif()
    if(DEFINED ANDROID_STL)
      list(APPEND cmake_configure_args "-DANDROID_STL=${ANDROID_STL}")
    endif()
    if(DEFINED ANDROID_TOOLCHAIN_NAME)
      list(APPEND cmake_configure_args "-DANDROID_TOOLCHAIN_NAME=${ANDROID_TOOLCHAIN_NAME}")
    endif()
  endif()
else()
  list(APPEND cmake_configure_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
endif()
set(fluent_bit_external_project_dir ${CMAKE_CURRENT_BINARY_DIR}/fluent_bit)
include(ExternalProject)
set(fluent_bit_version "1ab360f79c73627a0789551287c074aa2016bf26")
# message(${CMAKE_CURRENT_LIST_DIR})
ExternalProject_Add(fluent_bit_src
  GIT_REPOSITORY https://github.com/fluent/fluent-bit
  GIT_TAG ${fluent_bit_version}
  GIT_CONFIG advice.detachedHead=false
  PATCH_COMMAND patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/mpack.patch
  # Suppress git update due to https://gitlab.kitware.com/cmake/cmake/-/issues/16419
  UPDATE_COMMAND ""
  TIMEOUT 6000
  BUILD_IN_SOURCE On
  ${cmake_commands}
  CMAKE_ARGS
    -DBUILD_TESTING=Off
    -DCMAKE_INSTALL_PREFIX=${fluent_bit_external_project_dir}/install/
    -DCMAKE_INSTALL_LIBDIR=lib/
    -DFLB_EXAMPLES=Off
    -DFLB_OUT_PGSQL=On
    -DFLB_SHARED_LIB=On
    -DFLB_PROXY_GO=On
    ${cmake_configure_args}
)

externalproject_get_property(fluent_bit_src INSTALL_DIR)
set(fluent_bit_INCLUDE_DIR "${INSTALL_DIR}/../fluent_bit/install/include")
set(fluent_bit_LIB_DIR "${INSTALL_DIR}/../fluent_bit/install/lib/fluent-bit")

set(FB_SRC_LIB_INCLUDE_DIR  "${INSTALL_DIR}/src/fluent_bit_src/lib")

set(ares_INCLUDE_FILES
  "${FB_SRC_LIB_INCLUDE_DIR}/c-ares-1.18.1/include/ares.h"
  "${FB_SRC_LIB_INCLUDE_DIR}/c-ares-1.18.1/include/ares_build.h"
  "${FB_SRC_LIB_INCLUDE_DIR}/c-ares-1.18.1/include/ares_rules.h"
  "${FB_SRC_LIB_INCLUDE_DIR}/c-ares-1.18.1/include/ares_version.h"
)
set(cfl_INCLUDE_DIR "${FB_SRC_LIB_INCLUDE_DIR}/cfl/include/cfl")
set(cmetrics_INCLUDE_DIR "${FB_SRC_LIB_INCLUDE_DIR}/cmetrics/include/cmetrics")
set(ctraces_INCLUDE_DIR "${FB_SRC_LIB_INCLUDE_DIR}/ctraces/include/ctraces")
set(jsmn_INCLUDE_DIR "${FB_SRC_LIB_INCLUDE_DIR}/jsmn")
set(mkcore_INCLUDE_DIR "${FB_SRC_LIB_INCLUDE_DIR}/monkey/include/monkey/mk_core")
set(mpack_INCLUDE_DIR "${FB_SRC_LIB_INCLUDE_DIR}/mpack-amalgamation-1.1/src/mpack")
set(msgpack_INCLUDE_DIR "${FB_SRC_LIB_INCLUDE_DIR}/msgpack-c/include/msgpack")
set(msgpack_INCLUDE_FILES
  "${FB_SRC_LIB_INCLUDE_DIR}/msgpack-c/include/msgpack.h"
)
set(prometheus_remote_write_INCLUDE_DIR "${FB_SRC_LIB_INCLUDE_DIR}/cmetrics/include/prometheus_remote_write")
set(xxhash_INCLUDE_FILES
  "${FB_SRC_LIB_INCLUDE_DIR}/cfl/lib/xxhash/xxhash.h"
  "${FB_SRC_LIB_INCLUDE_DIR}/cfl/lib/xxhash/xxh3.h"
)

install(DIRECTORY ${fluent_bit_INCLUDE_DIR}/ DESTINATION include)
install(FILES ${ares_INCLUDE_FILES} DESTINATION include)
install(DIRECTORY ${cfl_INCLUDE_DIR}/ DESTINATION include/cfl)
install(DIRECTORY ${cmetrics_INCLUDE_DIR}/ DESTINATION include/cmetrics)
install(DIRECTORY ${ctraces_INCLUDE_DIR}/ DESTINATION include/ctraces)
install(DIRECTORY ${jsmn_INCLUDE_DIR}/ DESTINATION include/jsmn)
install(DIRECTORY ${mkcore_INCLUDE_DIR}/ DESTINATION include/mk_core)
install(DIRECTORY ${mpack_INCLUDE_DIR}/ DESTINATION include/mpack)
install(DIRECTORY ${msgpack_INCLUDE_DIR}/ DESTINATION include/msgpack)
install(FILES ${msgpack_INCLUDE_FILES} DESTINATION include)
install(DIRECTORY ${prometheus_remote_write_INCLUDE_DIR}/ DESTINATION include/prometheus_remote_write)
install(FILES ${xxhash_INCLUDE_FILES}/ DESTINATION include)

install(FILES ${fluent_bit_LIB_DIR}/libfluent-bit.so DESTINATION lib)
install(FILES ${INSTALL_DIR}/src/fluent_bit_src/library/libmsgpack-c-static.a DESTINATION lib)
install(FILES ${INSTALL_DIR}/src/fluent_bit_src/library/libfluent-bit.a DESTINATION lib)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY ${INSTALL_DIR}/src/fluent_bit_src/cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(libfluent-bit.so fluent-bit)
ament_package(CONFIG_EXTRAS fluentbit_vendor-extras.cmake)
