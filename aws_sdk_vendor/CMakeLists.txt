# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.17)
project(aws_sdk_vendor)

find_package(ament_cmake REQUIRED)
include(ExternalProject)

# Pinned to the tag verified working against RustFS (PutObject + multipart) before
# adoption — see the ADR-0007 / Phase 2 verification. Bump this by re-running the
# vendoring steps in ADR-0012, not by changing this value alone.
set(AWS_SDK_VERSION 1.11.600)

set(_aws_install "${CMAKE_CURRENT_BINARY_DIR}/aws_sdk_install")
set(_aws_source "${CMAKE_CURRENT_SOURCE_DIR}/vendor/aws-sdk-cpp")

# Build aws-cpp-sdk-core + aws-cpp-sdk-s3 (only) from the flattened, plain-committed
# source tree under vendor/aws-sdk-cpp (see ADR-0012) instead of a live `git clone` —
# the ROS buildfarm's binarydeb jobs run with no network access, so ExternalProject's
# GIT_REPOSITORY download step can never succeed there (#423 proved this fails with a
# clear network error against the old CMakeLists). SOURCE_DIR with no GIT_*/URL/
# DOWNLOAD_COMMAND option disables ExternalProject's download step entirely, so this
# is a plain in-tree source build.
#
# vendor/aws-sdk-cpp/crt/aws-crt-cpp is aws-sdk-cpp's one top-level submodule
# (crt/aws-crt-cpp), itself recursed except for the one nested submodule ADR-0012
# excludes (s2n's CBMC formal-verification model, needed only for `make cbmc`) and
# crt/aws-lc, which BUILD_DEPS's USE_OPENSSL=ON default never builds (aws-c-cal links
# system libcrypto instead — see ADR-0012). BUILD_DEPS=ON (the default) still vendors
# the rest of the aws-c-*/aws-crt-cpp dependency chain, so the only external system
# libraries needed remain curl/openssl/zlib (the package.xml build_depends). Zlib
# request compression is optional per the SDK and disabled to keep the build minimal
# (S3-compatible stores don't need it).
ExternalProject_Add(aws_sdk_cpp_ext
  SOURCE_DIR ${_aws_source}
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
  USES_TERMINAL_BUILD TRUE)

# Stage the built SDK into this package's ament install prefix, so downstream
# find_package(AWSSDK) resolves it via CMAKE_PREFIX_PATH and the ament environment hooks
# add <prefix>/lib to LD_LIBRARY_PATH at runtime. aws-sdk-cpp installs relocatable CMake
# configs, so copying the tree wholesale is sufficient.
install(DIRECTORY ${_aws_install}/ DESTINATION . USE_SOURCE_PERMISSIONS)

ament_package()
