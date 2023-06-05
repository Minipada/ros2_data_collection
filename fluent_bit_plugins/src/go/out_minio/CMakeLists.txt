cmake_minimum_required(VERSION 3.0)
project(out_minio)

set(GO_COMPILER go)
set(MINIO_LIBRARY out_minio.so)
set(MINIO_INCLUDE out_minio.h)

add_custom_target(plugin_out_minio ALL
    COMMAND ${GO_COMPILER} build -buildmode=c-shared -buildvcs=false -o ${MINIO_LIBRARY} ${CMAKE_CURRENT_SOURCE_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/${MINIO_LIBRARY}
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/${MINIO_INCLUDE}
  DESTINATION lib
)
