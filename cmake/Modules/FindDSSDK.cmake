###############################################################################
# Find SoftKinetic DepthSense SDK
#
#     find_package(DSSDK)
#
# Variables defined by this module:
#
#  DSSDK_FOUND                 True if DepthSense SDK was found
#  DSSDK_VERSION               The version of DepthSense SDK
#  DSSDK_INCLUDE_DIRS          The location(s) of DepthSense SDK headers
#  DSSDK_LIBRARIES             Libraries needed to use DepthSense SDK

find_path(DSSDK_DIR include/DepthSenseVersion.hxx
          PATHS "$ENV{DEPTHSENSESDK32}"
		        "$ENV{DEPTHSENSESDK64}"
		        "$ENV{PROGRAMFILES}/SoftKinetic/DepthSenseSDK"
				"$ENV{PROGRAMW6432}/SoftKinetic/DepthSenseSDK"
				"C:/Program Files (x86)/SoftKinetic/DepthSenseSDK"
				"C:/Program Files/SoftKinetic/DepthSenseSDK"
          DOC "DepthSense SDK directory")

if(DSSDK_DIR)

  # Include directories
  set(DSSDK_INCLUDE_DIRS ${DSSDK_DIR}/include)

  # Libraries
  set(DSSDK_LIBRARIES ${DSSDK_LIBRARY})
  find_library(DSSDK_LIBRARY
               NAMES DepthSense.lib
               PATHS "${DSSDK_DIR}/lib/" NO_DEFAULT_PATH)

  # Version
  set(DSSDK_VERSION 0)
  file(STRINGS "${DSSDK_INCLUDE_DIRS}/DepthSenseVersion.hxx" _dsversion_H_CONTENTS REGEX "#define DEPTHSENSE_FILE_VERSION_STRING.*")
  set(_DSSDK_VERSION_REGEX "([0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+)")
  if("${_dsversion_H_CONTENTS}" MATCHES ".*#define DEPTHSENSE_FILE_VERSION_STRING .*${_DSSDK_VERSION_REGEX}.*")
    set(DSSDK_VERSION "${CMAKE_MATCH_1}")
  endif()
  unset(_dsversion_H_CONTENTS)
  
  set(DSSDK_FOUND TRUE)

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DSSDK DEFAULT_MSG
                                  DSSDK_LIBRARY DSSDK_INCLUDE_DIRS)

mark_as_advanced(DSSDK_LIBRARY DSSDK_INCLUDE_DIRS)

if(DSSDK_FOUND)
  if(NOT DSSDK_FIND_QUIETLY)
    message(STATUS "DepthSense SDK found (include: ${DSSDK_INCLUDE_DIRS}, libs: ${DSSDK_LIBRARIES})")
    message(STATUS "DepthSense SDK version: ${DSSDK_VERSION}")	
  endif()
else()
  if(DSSDK_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find DepthSense SDK!")
  elseif(NOT DSSDK_FIND_QUIETLY)
    message(WARNING "Could not find DepthSense SDK!")
	return()
  endif()
endif()

if(MSVC)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /NODEFAULTLIB:LIBCMTD")
endif()