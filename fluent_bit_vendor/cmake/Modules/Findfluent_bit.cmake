if(NOT fluent_bit_ROOT_DIR AND DEFINED ENV{fluent_bit_ROOT_DIR})
    set(fluent_bit_ROOT_DIR "$ENV{fluent_bit_ROOT_DIR}" CACHE PATH
        "fluent_bit base directory location (optional, used for nonstandard installation paths)")
endif()

set(CMAKE_FIND_LIBRARY_SUFFIXES ".so" ".a")
if(fluent_bit_ROOT_DIR)
  set(fluent_bit_INCLUDE_PATH PATHS "${fluent_bit_ROOT_DIR}/include" NO_DEFAULT_PATH)
  set(fluent_bit_LIBRARY_PATH PATHS "${fluent_bit_ROOT_DIR}/lib/fluent-bit" NO_DEFAULT_PATH)
else()
  set(fluent_bit_INCLUDE_PATH "")
  set(fluent_bit_LIBRARY_PATH "")
endif()

# Search for headers and the library
find_path(fluent_bit_INCLUDE_DIR NAMES "fluent-bit.h" ${fluent_bit_INCLUDE_PATH})
find_library(fluent_bit_LIBRARY NAMES fluent-bit ${fluent_bit_LIBRARY_PATH})

mark_as_advanced(fluent_bit_INCLUDE_DIR fluent_bit_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  fluent_bit
  DEFAULT_MSG
  fluent_bit_INCLUDE_DIR
  fluent_bit_LIBRARY
)

if(${fluent_bit_FOUND})
  set(fluent_bit_INCLUDE_DIRS ${fluent_bit_INCLUDE_DIR} ${msgpack_INCLUDE_DIR} ${monkey_INCLUDE_DIR} ${cmetrics_INCLUDE_DIR} ${prometheus_remote_write_INCLUDE_DIR})
  set(fluent_bit_LIBRARIES ${fluent_bit_LIBRARY})

  add_library(fluent_bit::fluent_bit UNKNOWN IMPORTED)
  set_property(TARGET fluent_bit::fluent_bit PROPERTY IMPORTED_LOCATION ${fluent_bit_LIBRARY} ${msgpack_INCLUDE_DIR})
  set_property(TARGET fluent_bit::fluent_bit PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${fluent_bit_INCLUDE_DIR} ${msgpack_INCLUDE_DIR} ${monkey_INCLUDE_DIR} ${cmetrics_INCLUDE_DIR} ${prometheus_remote_write_INCLUDE_DIR})
  list(APPEND fluent_bit_TARGETS fluent_bit::fluent_bit)

endif()
