cmake_minimum_required(VERSION 3.8)
project(Bachelor)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    message(STATUS "GCC detected, adding compile flags")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")


add_subdirectory(visualizer)
add_subdirectory(src)
add_subdirectory(test)

# add_executable(Bachelor main.cpp)
# target_link_libraries(Bachelor visualizer)

# add_custom_command(
#     TARGET Bachelor
#     POST_BUILD COMMAND
#         ${CMAKE_COMMAND} -E copy_directory
#         "${CMAKE_CURRENT_SOURCE_DIR}/assets"
#         $<TARGET_FILE_DIR:Bachelor>/assets)





