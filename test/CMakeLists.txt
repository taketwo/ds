macro(TEST_ADD _name)
  set(options)
  set(one_value_args)
  set(multi_value_args LINK_WITH)
  cmake_parse_arguments(TEST_ADD "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})
  set(_executable test_${_name})
  add_executable(${_executable} ${_executable}.cpp)
  target_link_libraries(${_executable} ${GTEST_LIBRARIES} ${Boost_LIBRARIES} ${TEST_ADD_LINK_WITH} pthread)
  add_test(NAME ${_name} COMMAND ${_executable})
  add_dependencies(tests ${_executable})
endmacro(TEST_ADD)

find_package(GTest REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_custom_target(tests "${CMAKE_CTEST_COMMAND}" "-V" VERBATIM)

TEST_ADD(buffers
         LINK_WITH depth_sense
)
