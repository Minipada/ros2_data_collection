# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.17)  # GIT_SUBMODULES_RECURSE (see below)
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
#
# aws-sdk-cpp itself declares exactly one top-level submodule, crt/aws-crt-cpp (verified
# against its .gitmodules at AWS_SDK_VERSION); of aws-crt-cpp's own 13 submodules, only
# crt/s2n declares a further nested one — tests/cbmc/aws-verification-model-for-libcrypto,
# s2n-tls's CBMC formal-verification model repo, needed only for `make cbmc` and unrelated
# to building `-DBUILD_ONLY=s3` (#271). GIT_SUBMODULES_RECURSE has no way to exclude a
# single nested submodule while still recursing everything else it's nested under, so the
# top-level ExternalProject_Add step only checks out aws-crt-cpp itself (no recursion);
# PATCH_COMMAND then does the real recursive submodule init by hand, skipping just the
# CBMC one via a `git -c submodule.<name>.update=none` override — verified directly
# against the real upstream repos (a download-only ExternalProject run: every other
# submodule the SDK needs — including crt/s2n itself — gets checked out normally, and only
# tests/cbmc/aws-verification-model-for-libcrypto is reported "Skipping").
ExternalProject_Add(aws_sdk_cpp_ext
  GIT_REPOSITORY https://github.com/aws/aws-sdk-cpp.git
  GIT_TAG ${AWS_SDK_VERSION}
  GIT_SHALLOW ON
  GIT_SUBMODULES "crt/aws-crt-cpp"
  GIT_SUBMODULES_RECURSE OFF
  GIT_PROGRESS ON
  PATCH_COMMAND git -C crt/aws-crt-cpp
    -c submodule.tests/cbmc/aws-verification-model-for-libcrypto.update=none
    submodule update --init --recursive
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
