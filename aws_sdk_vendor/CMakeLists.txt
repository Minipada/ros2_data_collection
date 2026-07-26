cmake_minimum_required(VERSION 3.16)
project(aws_sdk_vendor)

find_package(ament_cmake REQUIRED)
include(ExternalProject)

# Pinned to the tag verified working against RustFS (PutObject + multipart) before
# adoption — see the ADR-0007 / Phase 2 verification. Bump this to move versions.
set(AWS_SDK_VERSION 1.11.600)

set(_aws_install "${CMAKE_CURRENT_BINARY_DIR}/aws_sdk_install")

# Build aws-cpp-sdk-core + aws-cpp-sdk-s3 (only) from source. BUILD_DEPS=ON (the default)
# vendors the aws-c-* / aws-crt-cpp dependency chain from the recursed submodules, so the
# only external system libraries needed are curl/openssl/zlib (the package.xml
# build_depends). Zlib request compression is optional per the SDK and disabled to keep
# the build minimal (S3-compatible stores don't need it).
ExternalProject_Add(aws_sdk_cpp_ext
  GIT_REPOSITORY https://github.com/aws/aws-sdk-cpp.git
  GIT_TAG ${AWS_SDK_VERSION}
  GIT_SHALLOW ON
  GIT_SUBMODULES_RECURSE ON
  GIT_PROGRESS ON
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${_aws_install}
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_ONLY=s3
    -DENABLE_TESTING=OFF
    -DAUTORUN_UNIT_TESTS=OFF
    -DBUILD_SHARED_LIBS=ON
    -DENABLE_ZLIB_REQUEST_COMPRESSION=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
  BUILD_BYPRODUCTS
    ${_aws_install}/lib/libaws-cpp-sdk-core.so
    ${_aws_install}/lib/libaws-cpp-sdk-s3.so
  USES_TERMINAL_DOWNLOAD TRUE
  USES_TERMINAL_BUILD TRUE)

# Stage the built SDK into this package's ament install prefix, so downstream
# find_package(AWSSDK) resolves it via CMAKE_PREFIX_PATH and the ament environment hooks
# add <prefix>/lib to LD_LIBRARY_PATH at runtime. aws-sdk-cpp installs relocatable CMake
# configs, so copying the tree wholesale is sufficient.
install(DIRECTORY ${_aws_install}/ DESTINATION . USE_SOURCE_PERMISSIONS)

ament_package()
