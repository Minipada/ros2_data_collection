# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(aws_sdk_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# Pinned to the tag verified working against RustFS (PutObject + multipart) before
# adoption — see the ADR-0007 / Phase 2 verification. Bump this to move versions.
set(AWS_SDK_VERSION 1.11.600)

# Live git fetch at colcon-build time via ament_cmake_vendor_package's ament_vendor()
# macro — the same idiom zmqpp_vendor/tinyxml_vendor/yaml_cpp_vendor already use
# successfully on the real ROS buildfarm (see docs/adr/0012). Its VCS_TYPE git path
# resolves through `vcs import --recursive`, which recurses aws-sdk-cpp's one
# top-level submodule (crt/aws-crt-cpp) and that submodule's own chain in full —
# including crt/s2n's unrelated CBMC formal-verification model, harmless to fetch
# with real network available and not worth excluding via a hand-rolled patch step.
#
# Build aws-cpp-sdk-core + aws-cpp-sdk-s3 (only). BUILD_DEPS=ON (the default) vendors
# the aws-c-* / aws-crt-cpp dependency chain from the recursed submodules, so the only
# external system libraries needed are curl/openssl/zlib (the package.xml
# build_depends). Zlib request compression is optional per the SDK and disabled to
# keep the build minimal (S3-compatible stores don't need it).
#
# GLOBAL_HOOK + an explicit CMAKE_BUILD_TYPE: dc_bridge (the real downstream
# consumer) never calls find_package(aws_sdk_vendor) itself — it only
# find_package(AWSSDK)s, expecting colcon to have already put this package's install
# prefix on CMAKE_PREFIX_PATH. GLOBAL_HOOK is what installs that as an environment
# hook (applied to every downstream package's build unconditionally) rather than a
# CMake config-extra a consumer would only pick up by find_package()-ing this vendor
# package first. Pinning CMAKE_BUILD_TYPE here (rather than relying on whatever the
# outer colcon invocation passes down) keeps the vendored SDK's own build
# deterministic regardless of how aws_sdk_vendor itself is invoked (e.g. the
# package-only `colcon build --packages-select aws_sdk_vendor` step in
# tools/e2e/Containerfile, which passes no --cmake-args of its own).
ament_vendor(aws_sdk_cpp_vendor
  GLOBAL_HOOK
  VCS_TYPE git
  VCS_URL https://github.com/aws/aws-sdk-cpp.git
  VCS_VERSION ${AWS_SDK_VERSION}
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_ONLY=s3
    -DENABLE_TESTING=OFF
    -DAUTORUN_UNIT_TESTS=OFF
    -DENABLE_ZLIB_REQUEST_COMPRESSION=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
)

ament_package()
