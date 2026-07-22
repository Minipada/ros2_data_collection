cmake_minimum_required(VERSION 3.18)
project(vector_vendor)

find_package(ament_cmake REQUIRED)

# Bumping Vector is a single-line change: update the version and the checksum
# for every supported architecture below (see ADR-0002).
set(VECTOR_VERSION "0.57.0")
set(VECTOR_SHA256_x86_64 "b04fafcffcbeb3d43cbba587583dbe3338b24dd34256b3967fb15d137aff75f2")
set(VECTOR_SHA256_aarch64 "81cec5343e49985351e13f76ae3d550d7b0f0ac64fd4d88c07e3873bdfdf100a")

# Override: point vector_path at a system-installed Vector binary to skip the
# download entirely (air-gapped builds, distro packages). May be set as a
# colcon cmake-arg (--cmake-args -Dvector_path=/usr/bin/vector) or via the
# VECTOR_PATH environment variable.
if(NOT DEFINED vector_path AND DEFINED ENV{VECTOR_PATH})
  set(vector_path "$ENV{VECTOR_PATH}")
endif()
set(vector_path "${vector_path}" CACHE FILEPATH
  "Path to a system-installed Vector binary. When set, vector_vendor vendors \
this binary instead of downloading one.")

if(vector_path)
  if(NOT EXISTS "${vector_path}")
    message(FATAL_ERROR "vector_vendor: vector_path='${vector_path}' does not exist")
  endif()
  set(_vector_binary "${vector_path}")
else()
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(_vector_arch "x86_64")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(_vector_arch "aarch64")
  else()
    message(FATAL_ERROR
      "vector_vendor: unsupported architecture '${CMAKE_SYSTEM_PROCESSOR}'; "
      "set vector_path to a system-installed Vector binary instead")
  endif()

  set(_vector_sha256 "${VECTOR_SHA256_${_vector_arch}}")
  if(NOT _vector_sha256)
    message(FATAL_ERROR "vector_vendor: no pinned checksum for architecture '${_vector_arch}'")
  endif()

  set(_vector_tarball_name "vector-${VECTOR_VERSION}-${_vector_arch}-unknown-linux-musl.tar.gz")
  set(_vector_url "https://github.com/vectordotdev/vector/releases/download/v${VECTOR_VERSION}/${_vector_tarball_name}")
  set(_vector_download_dir "${CMAKE_CURRENT_BINARY_DIR}/download")
  set(_vector_tarball "${_vector_download_dir}/${_vector_tarball_name}")

  file(MAKE_DIRECTORY "${_vector_download_dir}")

  set(_need_download TRUE)
  if(EXISTS "${_vector_tarball}")
    file(SHA256 "${_vector_tarball}" _cached_sha256)
    if(_cached_sha256 STREQUAL _vector_sha256)
      set(_need_download FALSE)
    endif()
  endif()

  if(_need_download)
    message(STATUS "vector_vendor: downloading ${_vector_url}")
    file(DOWNLOAD "${_vector_url}" "${_vector_tarball}"
      EXPECTED_HASH SHA256=${_vector_sha256}
      TLS_VERIFY ON
      STATUS _download_status
    )
    list(GET _download_status 0 _download_code)
    if(NOT _download_code EQUAL 0)
      list(GET _download_status 1 _download_message)
      file(REMOVE "${_vector_tarball}")
      message(FATAL_ERROR
        "vector_vendor: failed to download Vector ${VECTOR_VERSION} for "
        "${_vector_arch}: ${_download_message}")
    endif()
  endif()

  set(_vector_extract_dir "${CMAKE_CURRENT_BINARY_DIR}/extract")
  file(REMOVE_RECURSE "${_vector_extract_dir}")
  file(MAKE_DIRECTORY "${_vector_extract_dir}")
  file(ARCHIVE_EXTRACT INPUT "${_vector_tarball}" DESTINATION "${_vector_extract_dir}")

  file(GLOB _vector_binary_candidates "${_vector_extract_dir}/*/bin/vector")
  list(LENGTH _vector_binary_candidates _vector_binary_candidate_count)
  if(NOT _vector_binary_candidate_count EQUAL 1)
    message(FATAL_ERROR
      "vector_vendor: expected exactly one extracted vector binary in "
      "${_vector_extract_dir}, found ${_vector_binary_candidate_count}")
  endif()
  list(GET _vector_binary_candidates 0 _vector_binary)

  execute_process(
    COMMAND "${_vector_binary}" --version
    OUTPUT_VARIABLE _vector_version_output
    RESULT_VARIABLE _vector_version_result
  )
  if(NOT _vector_version_result EQUAL 0)
    message(FATAL_ERROR "vector_vendor: '${_vector_binary} --version' failed")
  endif()
  if(NOT _vector_version_output MATCHES "vector ${VECTOR_VERSION} ")
    message(FATAL_ERROR
      "vector_vendor: downloaded Vector reports version '${_vector_version_output}', "
      "expected ${VECTOR_VERSION}")
  endif()
endif()

install(
  PROGRAMS "${_vector_binary}"
  DESTINATION lib/${PROJECT_NAME}
  RENAME vector
)

ament_package()
