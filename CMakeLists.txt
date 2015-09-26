cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ds)

option(BUILD_TESTS "Build tests." OFF)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

find_package(PCL 1.7.2 REQUIRED)
find_package(DSSDK REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${DSSDK_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_library(depth_sense
  src/depth_sense/depth_sense_device_manager.cpp
  src/depth_sense/depth_sense_grabber_impl.cpp
  src/depth_sense_grabber.cpp
  src/io_exception.cpp
)

add_executable(depth_sense_viewer
  src/depth_sense_viewer.cpp
)
target_link_libraries(depth_sense_viewer
  depth_sense
  ${PCL_LIBRARIES}
  ${DSSDK_LIBRARIES}
)

add_executable(multi_grabber
  src/multi_grabber.cpp
)
target_link_libraries(multi_grabber
  depth_sense
  ${PCL_LIBRARIES}
  ${DSSDK_LIBRARIES}
)

if (BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

