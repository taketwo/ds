cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ds)

find_package(PCL 1.7 REQUIRED)

set(DEPTHSENSESDK_DIR /opt/softkinetic/DepthSenseSDK)
set(DEPTHSENSESDK_INCLUDE_DIR ${DEPTHSENSESDK_DIR}/include)
set(DEPTHSENSESDK_LIBRARY_DIR ${DEPTHSENSESDK_DIR}/lib)
set(DEPTHSENSESDK_LIBRARIES ${DEPTHSENSESDK_LIBRARY_DIR}/libDepthSense.so ${DEPTHSENSESDK_LIBRARY_DIR}/libDepthSensePlugins.so ${DEPTHSENSESDK_LIBRARY_DIR}/libturbojpeg.so ${DEPTHSENSESDK_LIBRARY_DIR}/libusb-1.0.so)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${DEPTHSENSESDK_INCLUDE_DIR}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${DEPTHSENSESDK_LIBRARY_DIR}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(depth_sense_viewer
  src/depth_sense/depth_sense_device_manager.cpp
  src/depth_sense_grabber.cpp
  src/depth_sense_viewer.cpp
)
target_link_libraries(depth_sense_viewer
  ${PCL_LIBRARIES}
  ${DEPTHSENSESDK_LIBRARIES}
)

add_executable(multi_grabber
  src/depth_sense/depth_sense_device_manager.cpp
  src/depth_sense_grabber.cpp
  src/multi_grabber.cpp
)
target_link_libraries(multi_grabber
  ${PCL_LIBRARIES}
  ${DEPTHSENSESDK_LIBRARIES}
)

