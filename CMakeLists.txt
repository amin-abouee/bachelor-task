cmake_minimum_required(VERSION 3.8)
project(Bachelor)

# set build type as release ny default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    message(STATUS "GCC detected, adding compile flags")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

if(CMAKE_BUILD_TYPE STREQUAL Debug)
	set (BUILD_PREFIX "-d")
else()
	set (BUILD_PREFIX "")
endif()

add_subdirectory(visualizer)
add_subdirectory(src)

find_package(GTest REQUIRED)
if(GTest_FOUND)
    add_subdirectory(test)
endif()





