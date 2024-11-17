cmake_minimum_required(VERSION 3.10)

project(graph_tests)

# Enable testing
enable_testing()

# FetchContent module for downloading dependencies
include(FetchContent)

# Download GoogleTest if not already present
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.11.0  # or any other tag you prefer
)
FetchContent_MakeAvailable(googletest)

# Add your source files here (ensure this path is correct)
file(GLOB_RECURSE TEST_SOURCES "*.cpp")

foreach(TEST_SOURCE ${TEST_SOURCES})
  get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

  # Create the test executable
  add_executable(${TEST_NAME} ${TEST_SOURCE})

  # Link the test executable with GoogleTest and the graph library
  target_link_libraries(${TEST_NAME} PRIVATE gtest gtest_main graph)

  # Add the test to CMake's test suite
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()

# Ensure that the tests are included in the final build
if (TARGET googletest)
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
endif()
