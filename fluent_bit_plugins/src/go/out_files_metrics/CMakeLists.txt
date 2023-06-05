cmake_minimum_required(VERSION 3.0)
project(out_files_metrics)

set(GO_COMPILER go)
set(FILES_METRICS_LIBRARY out_files_metrics.so)
set(FILES_METRICS_INCLUDE out_files_metrics.h)

add_custom_target(plugin_out_files_metrics ALL
    COMMAND ${GO_COMPILER} build -buildmode=c-shared -buildvcs=false -o ${FILES_METRICS_LIBRARY} ${CMAKE_CURRENT_SOURCE_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/${FILES_METRICS_LIBRARY}
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/${FILES_METRICS_INCLUDE}
  DESTINATION lib
)
