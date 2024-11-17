cmake_minimum_required(VERSION 3.10)
project(pibt)

# Check for ccache
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) 
endif()

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(libs)
add_subdirectory(apps)

